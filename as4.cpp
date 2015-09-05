
#include "includes.h"
#include "Bone.h"
#include "Kinematics.h"
#include "Cylinder.h"

#define PI 3.14159265

//****************************************************
// Some Classes
//****************************************************

class Viewport;

class Viewport {
public:
	int w, h; // width and height
};


//****************************************************
// Global Variables
//****************************************************
Viewport viewport;
int currColor=0;
bool shading=0, wireframe=1, octopus=0;
GLuint object;
GLfloat lights[5][4]= {{0.9, 0.9, 0.9, 0.8}, {0.4, 0.0, 0.8, 0.8}, {0.0, 0.6, 0.6, 0.8}, {0.5, 0.5, 0.1, 0.8}, {0.0, 0.0, 0.4, 0.8}};
std::vector<std::vector<Bone> > appendages;
Kinematics test(0.1,0.01);
typedef Eigen::Vector3d (* ParametricFunctions) (float t);

Eigen::Vector3d heart(float t){return Eigen::Vector3d((16*pow(sin(t),3))/3, (13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t))/3, -3);}

Eigen::Vector3d spiral(float t){return Eigen::Vector3d(0.5*t*cos(2*PI*t), 0.5*t*sin(2*PI*t), t);}

Eigen::Vector3d figureEight(float t){return Eigen::Vector3d(2*cos(t), 2*sin(2*t), 1);}

Eigen::Vector3d butterfly(float t){return Eigen::Vector3d(sin(t)*(exp(cos(t))-2*cos(4*t)-pow(sin(t/12),5)),
														  cos(t)*(exp(cos(t))-2*cos(4*t)-pow(sin(t/12),5)), 1);}

Eigen::Vector3d tentacle(float t){return Eigen::Vector3d(0.001, 0.001, 6*cos(0.5*PI*t)+6);}
ParametricFunctions coolShapes[] = {spiral, heart, figureEight, butterfly, tentacle};
int sizeShapes = 5;
int shape = 0;
float stepSize = 0;
std::vector<Eigen::Vector3d> goals;

//****************************************************
// reshape viewport if the window is resized
//****************************************************
void myReshape(int w, int h) {
	viewport.w = w;
	viewport.h = h;
	glViewport (0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30, (GLfloat) w/(GLfloat) h, 1.0, 100.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(10, 10, 10, 0, 0, 0, 0, 0, 1);
}

//****************************************************
// Simple init function
//****************************************************
void initScene(){
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	glEnable(GL_DEPTH_TEST);
    //glEnable(GL_LIGHTING);

    glLightfv(GL_LIGHT0, GL_AMBIENT, lights[0]);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lights[0]);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lights[0]);
    GLfloat pos[] = {2000,2000,2000};
    glLightfv(GL_LIGHT0, GL_POSITION, pos);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_FLAT);

	object = glGenLists(1);
    for (int i = 0; i < appendages.size(); i++) {
        test.solveFK(appendages[i],0, 0.001, 0);
    }

	glNewList(object, GL_COMPILE);

	glLineWidth(1);
	glBegin(GL_LINES);
    
    glColor3f(1, 0, 0);
	glVertex3f(0,0,0);
	glVertex3f(100,0,0);

	glColor3f(0, 1, 0);
	glVertex3f(0,0,0);
	glVertex3f(0,100,0);

	glColor3f(0, 0, 1);
	glVertex3f(0,0,0);
	glVertex3f(0,0,100);
    
    glEnd();

    glBegin(GL_LINE_STRIP);
    float t = 0.0f;
    while (t<16) {
    	Eigen::Vector3d temp = coolShapes[shape](t);
    	glVertex3f(temp[0], temp[1], temp[2]);
        t += 0.01f;
    }
    glEnd();
    if (octopus) {    
        glPushMatrix();
        glScalef(1.0f,1.0f,1.5f);
        glutSolidSphere(1, 10, 10);
        glPopMatrix();
    }

    glEndList();

	glClearColor(0.0, 0.0, 0.0, 0.0);
}

void interpolateGoal(std::vector<Eigen::Vector3d> & goals, int arm) {
    static float t = 0.0f;
    t += stepSize;
    if (octopus) {
	    goals[arm] = coolShapes[shape](t) + Eigen::Vector3d(cos((2*PI/goals.size())*arm)+0.05*((float) rand()/(float) RAND_MAX), sin((2*PI/goals.size())*arm), 0);
    } else {
        goals[arm] = coolShapes[shape](t);
    }
}

void renderIK() {
    //Eigen::Vector3d goal = interpolateGoal();
    for (int i = 0; i < appendages.size(); i++) {
        test.solveIK(appendages[i], goals[i]);
        if (octopus) {
            glPushMatrix();
            glTranslatef(cos((2*PI/goals.size())*i), sin((2*PI/goals.size())*i), 0);
        }

        glColor3f(0, 1, 1);
        renderCylinder_convenient(0, 0, 0, appendages[i][0].currPos[0], appendages[i][0].currPos[1], appendages[i][0].currPos[2], 0.04, 10);

        glPushMatrix();
        glTranslatef(appendages[i][0].currPos[0], appendages[i][0].currPos[1], appendages[i][0].currPos[2]);
        glutSolidSphere(0.1, 10, 10);
        glPopMatrix();

        for (int j=1; j<appendages[i].size(); j++) {
            if (j%2==0) {
                glColor3f(0, 1, 1);
            } else {
                glColor3f(1, 0, 1);
            }
            renderCylinder_convenient(appendages[i][j-1].currPos[0], appendages[i][j-1].currPos[1], appendages[i][j-1].currPos[2], appendages[i][j].currPos[0], appendages[i][j].currPos[1], appendages[i][j].currPos[2], 0.04, 5);
            
            glPushMatrix();
            glTranslatef(appendages[i][j].currPos[0], appendages[i][j].currPos[1], appendages[i][j].currPos[2]);
            glutSolidSphere(0.1, 10, 10);
            glPopMatrix();
        }
        glBegin(GL_LINES);
        glColor3f(1, 1, 0);
        glVertex3f(0,0,0);
        glVertex3f(goals[i][0],goals[i][1],goals[i][2]);
        glEnd();

        if (octopus) glPopMatrix();
    }
}

//****************************************************
// function that does the actual drawing of stuff
//***************************************************
void myDisplay() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); glCallList(object);
    renderIK();
	glFlush();
	glutSwapBuffers();  
}
void myIdle() {
    for (int i = 0; i<appendages.size(); i++) {
        interpolateGoal(goals, i);
        glutPostRedisplay();
    }
}
void keyboard(unsigned char key, int x, int y)
{
	switch (key) {
	case 'n':
		shape = (shape+1) % sizeShapes;
		initScene();
		glutPostRedisplay();
		break;
    case 'u':
        stepSize += 0.005f;
        break;
    case 'i':
        stepSize -= 0.005f;
        break;
	case 's':
		if (shading) { 
			glShadeModel(GL_FLAT);
		} else {
			glShadeModel(GL_SMOOTH);
		}
		glutPostRedisplay();
		shading = !shading;
		break;
	case 'w':
		if (wireframe) { 
			glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		} else {
			glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		}
		glutPostRedisplay();
		wireframe = !wireframe;
		break;
	case '+':
		glScalef(1.1,1.1,1.1);
		glutPostRedisplay();
		break;
	case '-':
		glScalef(1/1.1,1/1.1,1/1.1);
		glutPostRedisplay();
		break;
	case 'c':
		currColor++;
		glLightfv(GL_LIGHT0, GL_AMBIENT, lights[currColor%5]);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lights[currColor%5]);
		glLightfv(GL_LIGHT0, GL_SPECULAR, lights[currColor%5]);
		glutPostRedisplay();
		break;
	case 27:
		exit(0);
		break;
	}
}

void arrows(int key, int x, int y)
{
	switch (key) {
	case GLUT_KEY_UP:
		glRotatef(15.,1.0,0.0,0.0);
		glutPostRedisplay();
		break;
	case GLUT_KEY_DOWN:
		glRotatef(-15.,1.0,0.0,0.0);
		glutPostRedisplay();
		break;
	case GLUT_KEY_LEFT:
		glRotatef(15.,0.0,1.0,0.0);
		glutPostRedisplay();
		break;
	case GLUT_KEY_RIGHT:
		glRotatef(-15.,0.0,1.0,0.0);
		glutPostRedisplay();
		break;
	}
}

//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
    if (argc > 1) {
        octopus = 1;
    }
    if (octopus) {
        for (int i = 0; i<8; i++) {
            std::vector<Bone> arm;
            for (int j=0; j<10; j++) {
                arm.push_back(Bone(0.3f));
            }
            appendages.push_back(arm);
            goals.push_back(coolShapes[shape](0));
        }
    } else {
        for (int i = 0; i<1; i++) {
            std::vector<Bone> arm;
            arm.push_back(Bone(2.0f));
            arm.push_back(Bone(1.0f));
            arm.push_back(Bone(1.3f));
            arm.push_back(Bone(0.5f));
            arm.push_back(Bone(0.8f));
            arm.push_back(Bone(0.3f));
            arm.push_back(Bone(1.0f));
            appendages.push_back(arm);
            goals.push_back(coolShapes[shape](0));
        }
    }

    //This initializes glut
	glutInit(&argc, argv);

	//This tells glut to use a double-buffered window with red, green, and blue channels 
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

	// Initalize theviewport size
	viewport.w = 640;
	viewport.h = 480;

	//The size and position of the window
	glutInitWindowSize(viewport.w, viewport.h);
	glutInitWindowPosition(0,0);
	glutCreateWindow(argv[0]);

	initScene();							// quick function to set up scene

	glutDisplayFunc(myDisplay);				// function to run when its time to draw something
    glutIdleFunc(myIdle);
	glutReshapeFunc(myReshape);				// function to run when the window gets resized
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(arrows);
	glutMainLoop();							// infinite loop that will keep drawing and resizing

	return 0;
}
