#ifdef _WIN32
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#ifndef __APPLE__
#include <GL/gl.h>
#include <GL/glut.h>
#else
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#endif
#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <AR/matrix.h>

//added myself
//#include <iostream>
//using namespace std;
//end of added myself
/* set up the video format globals */

#ifdef _WIN32
char			*vconf = "Data\\WDM_camera_flipV.xml";
#else
char			*vconf = "";
#endif

int             xsize, ysize;
int             thresh = 200;
int             count = 0;

int             mode = 1;

char           *cparam_name    = "Data/camera_para.dat";
ARParam         cparam;

char           *patt_name      = "Data/patt.id";
int             patt_id;
int             patt_width     = 80.0;
double          patt_center[2] = {0.0, 0.0};
double          patt_trans[3][4];

int mouseClicked = 0;
int click_x;
int click_y;

//serial port stuff
HANDLE hSerial;
DCB dcbSerialParams = { 0 };
COMMTIMEOUTS timeouts = { 0 };

static void   init(void);
static void   cleanup(void);
static void   mouseEvent(int button, int state, int x, int y);
static void   keyEvent( unsigned char key, int x, int y);
static void   mainLoop(void);
static void   draw( double trans[3][4] );
static void openSerialConnection(void);
static void closeSerialConnection(void);

int main(int argc, char **argv)
{
	glutInit(&argc, argv);
    init();

    arVideoCapStart();

	openSerialConnection();
    argMainLoop( mouseEvent, keyEvent, mainLoop );
	closeSerialConnection();
	return (0);
}

static void mouseEvent(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
	{
		printf("Left Mouse Button has been lifted. \n");
		printf("Position (x,y) = (%d,%d)\n", x, y);
		mouseClicked = 1;
		click_x = x;
		click_y = y;
	}
}

static void   keyEvent( unsigned char key, int x, int y)
{
    /* quit if the ESC key is pressed */
    if( key == 0x1b ) {
        printf("*** %f (frame/sec)\n", (double)count/arUtilTimer());
        cleanup();
        exit(0);
    }

    if( key == 'c' ) {
        printf("*** %f (frame/sec)\n", (double)count/arUtilTimer());
        count = 0;

        mode = 1 - mode;
        if( mode ) printf("Continuous mode: Using arGetTransMatCont.\n");
         else      printf("One shot mode: Using arGetTransMat.\n");
    }
}

/* main loop */
static void mainLoop(void)
{
    static int      contF = 0;
    ARUint8         *dataPtr;
    ARMarkerInfo    *marker_info;
    int             marker_num;
    int             j, k;

    /* grab a vide frame */
    if( (dataPtr = (ARUint8 *)arVideoGetImage()) == NULL ) {
        arUtilSleep(2);
        return;
    }
    if( count == 0 ) arUtilTimerReset();
    count++;

    argDrawMode2D();
    argDispImage( dataPtr, 0,0 );

    /* detect the markers in the video frame */
    if( arDetectMarker(dataPtr, thresh, &marker_info, &marker_num) < 0 ) {
        cleanup();
        exit(0);
    }

    arVideoCapNext();

    /* check for object visibility */
    k = -1;
    for( j = 0; j < marker_num; j++ ) {
        if( patt_id == marker_info[j].id ) {
            if( k == -1 ) k = j;
            else if( marker_info[k].cf < marker_info[j].cf ) k = j;
        }
    }
    if( k == -1 ) {
        contF = 0;
        argSwapBuffers();
        return;
    }

    /* get the transformation between the marker and the real camera */
    if( mode == 0 || contF == 0 ) {
        arGetTransMat(&marker_info[k], patt_center, patt_width, patt_trans);
		
    }
    else {
        arGetTransMatCont(&marker_info[k], patt_trans, patt_center, patt_width, patt_trans);
    }
    contF = 1;
	//printing out marker information for testing
	double          cam_trans_fin[3][4];
	arUtilMatMul(cparam.mat, patt_trans, cam_trans_fin);
	FILE *f = fopen("log.txt", "w");

	ARMat *m = arMatrixAlloc(3,3);
	m->m[0] = cam_trans_fin[0][0];
	m->m[1] = cam_trans_fin[0][1];
	m->m[2] = cam_trans_fin[0][3];
	m->m[3] = cam_trans_fin[1][0];
	m->m[4] = cam_trans_fin[1][1];
	m->m[5] = cam_trans_fin[1][3];
	m->m[6] = cam_trans_fin[2][0];
	m->m[7] = cam_trans_fin[2][1];
	m->m[8] = cam_trans_fin[2][3];

	if (mouseClicked){
		ARMat *v = arMatrixAlloc(3, 1);
		v->m[0] = (double)click_x;
		v->m[1] = (double)click_y;		
		v->m[2] = 1.0;
		arMatrixSelfInv(m);
		ARMat *r = arMatrixAllocMul(m, v);

		//assign the angle to a variable, if this angle = 0, print true
		double angle = 180.0 / 3.14159*atan2(r->m[0] / r->m[2], r->m[1] / r->m[2]);
		double euclideanDistance = r->m[0] * r->m[0] + r->m[1] * r->m[1];
		//printf("%f\n", angle);

		char command_right[1], command_left[1], command_stop[1], command_forward[1];
		command_right[0] = 'd';
		command_left[0] = 'a';
		command_stop[0] = ' ';
		command_forward[0] = 'w';
		DWORD bytes_written;
		int angleOnTarget = 0;
		if (euclideanDistance >= 0.001) {
			if (angle >= 12.5) {
				//printf("Sending right command...");
				if (!WriteFile(hSerial, command_right, 1, &bytes_written, NULL))
				{
					printf("Error\n");
					CloseHandle(hSerial);
					return 1;
				}
				Sleep(100);
			}
			else if (angle <= -12.5) {
				//printf("Sending left command...");
				if (!WriteFile(hSerial, command_left, 1, &bytes_written, NULL))
				{
					printf("Error\n");
					CloseHandle(hSerial);
					return 1;
				}
				Sleep(100);
			}
			else {
				angleOnTarget = 1;
				WriteFile(hSerial, command_forward, 1, &bytes_written, NULL);
				printf("Euclidean distance = %f\n", euclideanDistance);
			}
		}
		else {
			WriteFile(hSerial, command_stop, 1, &bytes_written, NULL);
		}
	}

	fclose(f);
	//end of testing-BRANCH
    draw( patt_trans );

    argSwapBuffers();
}

static void init( void )
{
    ARParam  wparam;

    /* open the video path */
    if( arVideoOpen( vconf ) < 0 ) exit(0);
    /* find the size of the window */
    if( arVideoInqSize(&xsize, &ysize) < 0 ) exit(0);
    printf("Image size (x,y) = (%d,%d)\n", xsize, ysize);

    /* set the initial camera parameters */
    if( arParamLoad(cparam_name, 1, &wparam) < 0 ) {
        printf("Camera parameter load error !!\n");
        exit(0);
    }
    arParamChangeSize( &wparam, xsize, ysize, &cparam );
    arInitCparam( &cparam );
    printf("*** Camera Parameter ***\n");
    arParamDisp( &cparam );

    if( (patt_id=arLoadPatt(patt_name)) < 0 ) {
        printf("pattern load error !!\n");
        exit(0);
    }

    /* open the graphics window */
    argInit( &cparam, 1.0, 0, 0, 0, 0 );
}

/* cleanup function called when program exits */
static void cleanup(void)
{
    arVideoCapStop();
    arVideoClose();
    argCleanup();
}

static void draw( double trans[3][4] )
{
    double    gl_para[16];
    GLfloat   mat_ambient[]     = {0.0, 0.0, 1.0, 1.0};
    GLfloat   mat_flash[]       = {0.0, 0.0, 1.0, 1.0};
    GLfloat   mat_flash_shiny[] = {50.0};
    GLfloat   light_position[]  = {100.0,-200.0,200.0,0.0};
    GLfloat   ambi[]            = {0.1, 0.1, 0.1, 0.1};
    GLfloat   lightZeroColor[]  = {0.9, 0.9, 0.9, 0.1};
    
    argDrawMode3D();
    argDraw3dCamera( 0, 0 );
    glClearDepth( 1.0 );
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    
    /* load the camera transformation matrix */
    argConvGlpara(trans, gl_para);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd( gl_para );

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambi);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_flash);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_flash_shiny);	
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMatrixMode(GL_MODELVIEW);
    glTranslatef( 0.0, 0.0, 25.0 );
    glutSolidCube(50.0);
    glDisable( GL_LIGHTING );

    glDisable( GL_DEPTH_TEST );
}

static void openSerialConnection(void) {
	printf("Opening Serial Port...");
	hSerial = CreateFile("\\\\.\\COM6", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hSerial == INVALID_HANDLE_VALUE) {
		printf("Error\n");
		//return 1;
	}
	else { printf("OK\n"); }
	//Set device parameters (38400 baud, 1 start bit, 1 stop bit, no parity)
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (GetCommState(hSerial, &dcbSerialParams) == 0)
	{
		printf("Error getting device state\n");
		CloseHandle(hSerial);
		//return 1;
	}
	//dcbSerialParams.BaudRate = CBR_38400;
	//changed to the same baud rate as the serial in arduino sketch
	dcbSerialParams.BaudRate = CBR_9600;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if (SetCommState(hSerial, &dcbSerialParams) == 0)
	{
		printf("Error setting device parameters\n");
		CloseHandle(hSerial);
		//return 1;
	}
	// Set COM port timeout settings
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	if (SetCommTimeouts(hSerial, &timeouts) == 0)
	{
		printf("Error setting timeouts\n");
		CloseHandle(hSerial);
		//return 1;
	}
}

static void closeSerialConnection(void) {
	printf("Closing serial port...");
	if (CloseHandle(hSerial) == 0)
	{
		printf("Error\n");
		//return 1;
	}
	printf("OK\n");
}
