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


static void   init(void);
static void   cleanup(void);
static void   mouseEvent(int button, int state, int x, int y);
static void   keyEvent( unsigned char key, int x, int y);
static void   mainLoop(void);
static void   draw( double trans[3][4] );

int main(int argc, char **argv)
{
	glutInit(&argc, argv);
    init();

    arVideoCapStart();
    argMainLoop( mouseEvent, keyEvent, mainLoop );
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

	fprintf(f, "start: patt_trans\n");
	fprintf(f, "%f %f %f %f\n", patt_trans[0][0], patt_trans[0][1], patt_trans[0][2], patt_trans[0][3]);
	fprintf(f, "%f %f %f %f\n", patt_trans[1][0], patt_trans[1][1], patt_trans[1][2], patt_trans[1][3]);
	fprintf(f, "%f %f %f %f\n", patt_trans[2][0], patt_trans[2][1], patt_trans[2][2], patt_trans[2][3]);
	fprintf(f, "end\n");
	fprintf(f, "start: cam_parameters\n");
	fprintf(f, "%f %f %f %f\n", cparam.mat[0][0], cparam.mat[0][1], cparam.mat[0][2], cparam.mat[0][3]);
	fprintf(f, "%f %f %f %f\n", cparam.mat[1][0], cparam.mat[1][1], cparam.mat[1][2], cparam.mat[1][3]);
	fprintf(f, "%f %f %f %f\n", cparam.mat[2][0], cparam.mat[2][1], cparam.mat[2][2], cparam.mat[2][3]);
	fprintf(f, "end\n");


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

		fprintf(f, "start: clicked point on ground\n");
		fprintf(f, "%f %f\n", r->m[0] / r->m[2], r->m[1] / r->m[2]);
		//printf( "%f %f\n", r->m[0] / r->m[2], r->m[1] / r->m[2]);
		//printf("%f\n", 180.0/3.14159*atan2(r->m[0] / r->m[2], r->m[1] / r->m[2]));
		fprintf(f, "end\n");
		//assign the angle to a variable, if this angle = 0, print true
		double angle = 180.0 / 3.14159*atan2(r->m[0] / r->m[2], r->m[1] / r->m[2]);
		printf("%f\n", angle);
		int angleOnTarget = 0;
		if (angle < 2.5 && angle > -2.5) {
			angleOnTarget = 1;
			printf("The angle has just reached 0, so angleOnTarget = %f \n", angleOnTarget);
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
