#include "Clock.h"
#include "Scene.h"
#include "Collisions.h"

Clock* times = new Clock;
real fps = 1.0f / 60.0f;
real counterFps  = 0.0f;

Scene scene(Vec2(0, 350), 100);

int numScene = 1;

bool boolShape = true;

Circle* cPrincipal = new Circle(Vec2(0, 0), 60, 0.0f);
OBB* bPrincipal = new OBB(Vec2(0, 0), Vec2(60, 60), 0.0f); 
Shape* principal = cPrincipal;

const int widthGlobal = 800;
const int heigtGlobal = 600;;

void DrawText(int x, int y, char *string)
{
	int len, i;

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);
	gluOrtho2D(0, w, h, 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glRasterPos2i(x, y);
	len = (int) strlen(string);
	for (i = 0; i < len; i++) 
	{
		glutBitmapCharacter(GLUT_BITMAP_9_BY_15, string[i]);
	}
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}



void TestDetectionCollision(Shape* p)
{
	Circle cLocal(Vec2(180, 300), 150, 0.0f);
	OBB bLocal(Vec2(570, 300), Vec2(180, 100), PI / 9.0f);

	Manifold m1(p, &cLocal);
	Manifold m2(p, &bLocal);

	if(m1.numContacts > 0) DrawPoint(m1.contacts[0].position);

	if(m2.numContacts > 0)
	{
		for(int i = 0; i < m2.numContacts; i++) DrawPoint(m2.contacts[i].position);
	}

	bLocal.DrawShape();
	cLocal.DrawShape();
	principal->DrawShape();
}



void TestLowFriction(void)
{
	OBB* wall1 = new OBB(Vec2(400, 530), Vec2(550, 10), 0.0f);
	wall1->CalculateMassInertia(0.0f);
	scene.Add(wall1);

	OBB* wall2 = new OBB(Vec2(220, 100), Vec2(250, 10), 15 * -RAD);
	wall2->CalculateMassInertia(0.0f);
	scene.Add(wall2);

	OBB* wall3 = new OBB(Vec2(500, 250), Vec2(265, 10), 15 *  RAD);
	wall3->CalculateMassInertia(0.0f);
	scene.Add(wall3);

	OBB* wall4 = new OBB(Vec2(130, 400), Vec2(200, 10), 10 * -RAD);
	wall4->CalculateMassInertia(0.0f);
	scene.Add(wall4);

	Circle* cir = new Circle(Vec2(85, 12), 20, 0.0f);
	cir->CalculateMassInertia(1.0f);
	cir->body->d = 0.35;
	scene.Add(cir);

	OBB* obb = new OBB(Vec2(23, 12), Vec2(20, 20), 15 * -RAD);
	obb->CalculateMassInertia(1.0f);
	obb->body->u = 0.018f; 
	scene.Add(obb);
}



void TestStacking(void)
{
	OBB* wall1 = new OBB(Vec2(400, 505), Vec2(400, 20), 0.0f);
	wall1->CalculateMassInertia(0.0f);
	scene.Add(wall1);

	for(int i = 0; i < 10; i++)
	{
		OBB* obb = new OBB(Vec2(250, 465 - 40 * i), Vec2(20, 20), 0.0f);
		obb->CalculateMassInertia(1.0f);	
		scene.Add(obb);

		Circle* cir = new Circle(Vec2(400, 465 - 40 * i), 20, 0.0f);
		cir->CalculateMassInertia(1.0f);	
		scene.Add(cir);

		if(i & 1) 
		{
			OBB* obb = new OBB(Vec2(550, 465 - 40 * i), Vec2(20, 20), 0.0f);
			obb->CalculateMassInertia(1.0f);	
			scene.Add(obb);
		}
		else
		{
			Circle* cir = new Circle(Vec2(550, 465 - 40 * i), 20, 0.0f);
			cir->CalculateMassInertia(1.0f);	
			scene.Add(cir);
		}
	}
}


void TestPyramid(void)
{
	OBB* wall1 = new OBB(Vec2(400, 505), Vec2(400, 20), 0.0f);
	wall1->CalculateMassInertia(0.0f);
	scene.Add(wall1);

	int lv = 16; real wh = 15;

	for(int i = 0; i < lv; i++)
	{
		for(int j = 0; j < lv - i; j++)
		{
			OBB* b = new OBB(Vec2(180 + wh * (i + j * 2) , 472 - wh * i * 2), Vec2(wh, wh), 0.0f);
			b->CalculateMassInertia(1.0f);
			scene.Add(b);
		}
	}
}



void TestAngryBirds(void)
{
	Circle* pork1 = new Circle(Vec2(475, 155), 10, 0);
	pork1->CalculateMassInertia(1.0f);
	scene.Add(pork1);

	Circle* pork2 = new Circle(Vec2(475, 325), 10, 0);
	pork2->CalculateMassInertia(1.0f);
	scene.Add(pork2);

	Circle* pork3 = new Circle(Vec2(395, 380), 10, 0);
	pork3->CalculateMassInertia(1.0f);
	scene.Add(pork3);

	OBB* wall1 = new OBB(Vec2(400, 505), Vec2(800, 20), 0.0f);
	wall1->CalculateMassInertia(0.0f);
	scene.Add(wall1);

	OBB* wall2 = new OBB(Vec2(280, 488), Vec2(50, 15), 20 * RAD);
	wall2->CalculateMassInertia(0.0f);
	scene.Add(wall2);

	OBB* wall3 = new OBB(Vec2(472, 476), Vec2(150, 20), 0.0f);
	wall3->CalculateMassInertia(0.0f);
	scene.Add(wall3);

	OBB* wall4 = new OBB(Vec2(664.5, 488), Vec2(50, 15), -20 * RAD);
	wall4->CalculateMassInertia(0.0f);
	scene.Add(wall4);
	
	for(int i = 0; i < 3; i++)
	{
		int temp = (i & 1)? 0 : 1;

		for(int j = 0; j < 7 - temp; j++)
		{
			OBB* wood1 = new OBB(Vec2(375 + 80 * i, 430 - 56 * j), Vec2(4.0f, 25), 0.0f);
			wood1->CalculateMassInertia(1.0f);
			scene.Add(wood1);

			OBB* wood2 = new OBB(Vec2(415 + 80 * i, 430 - 56 * j), Vec2(4.0f, 25), 0.0f);
			wood2->CalculateMassInertia(1.0f);
			scene.Add(wood2);

			OBB* wood3 = new OBB(Vec2(395 + 80 * i, 401 - 56 * j), Vec2(25, 4.0f), 0.0f);
			wood3->CalculateMassInertia(1.0f);		
			scene.Add(wood3);
		}
	}
}


void (*TestScene[])(void) = {TestLowFriction, TestStacking, TestPyramid, TestAngryBirds};

char* SceneStrings[] = 
{
	"Scene 1: Collision Detection", 
	"Scene 2: Low Friction",
	"Scene 3: Stacking Boxes And Circles",
	"Scene 4: Pyramid", 
	"Scene 5: Game Style Angry Bird",
};


void Start(void)
{
	scene.Clear();
	
	bPrincipal->CalculateMassInertia(0.0f);
	cPrincipal->CalculateMassInertia(0.0f);

	scene.Add(principal);

	if(numScene > 1)
	{
		TestScene[numScene - 2]();
	}
}




void Update(void)
{
	glClear(GL_COLOR_BUFFER_BIT);

	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

	DrawText(1, 15, SceneStrings[numScene - 1]);

	if(numScene == 1)
	{
		TestDetectionCollision(principal);
	}
	else
	{
		for(times->Elapsed(counterFps); counterFps >= fps; counterFps -= fps)
		{		
			scene.Step(fps);
		}

		for(auto temp : scene.bodies) temp->DrawShape();

		for(auto temp = scene.manifolds.begin(); temp != scene.manifolds.end(); temp++)
		{
			for(int i = 0; i < temp->second.numContacts; i++) DrawPoint(temp->second.contacts[i].position);
		}
	}

	glutSwapBuffers();
}


void MouseMotion(int x, int y)
{
	x += (widthGlobal - glutGet(GLUT_WINDOW_WIDTH))  * 0.5f;
	y += (heigtGlobal - glutGet(GLUT_WINDOW_HEIGHT)) * 0.5f;

	principal->body->X.Set(x, y);
}



void Keyboard(unsigned char key, int x, int y)
{
	x += (widthGlobal - glutGet(GLUT_WINDOW_WIDTH))  * 0.5f;
	y += (heigtGlobal - glutGet(GLUT_WINDOW_HEIGHT)) * 0.5f;

	switch (key)
	{
		case 'q':
		case 'Q': principal->body->θ += PI / 72;
		break;
		case 'e': 
		case 'E': principal->body->θ -= PI / 72;
		break;
		case 'l':
		case 'L': {glutPostRedisplay();}
		break;
		case 'b':
		case 'B':
		{
			OBB* temp = new OBB(Vec2(x, y), Vec2(20, 20), 0.0f);
			temp->CalculateMassInertia(1.0f);
			scene.Add(temp);
		}
		break;
		case 'c':
		case 'C':
		{
			Circle* temp = new Circle(Vec2(x, y), 20, 0.0f);
			temp->CalculateMassInertia(1.0f);
			scene.Add(temp);
		}
		break;
		case 'w':
		case 'W':
		{
			if(boolShape)
			{
				bPrincipal->body->X = principal->body->X;
				principal = bPrincipal;
			}
			else
			{
				cPrincipal->body->X = principal->body->X;
				principal = cPrincipal;
			}
			boolShape = !boolShape;
			scene.bodies[0] = principal;
		}
		break;
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		{
			numScene = int(key) - 48; 
			glutPostRedisplay();
		}
		break;
	}
}




void Reshape(int width, int height)
{
	if (!height) height = 1;
	glViewport(0, 0, width, height);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, width, height, 0);
	glTranslatef((width - widthGlobal) * 0.5f, (height - heigtGlobal) * 0.5f, 0.0f);
}



int main(int args, char** argv)
{	
	glutInit(&args, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowPosition(0,0);
	glutInitWindowSize(widthGlobal, heigtGlobal);
	glutCreateWindow("Ciroob Engine");
	glutReshapeFunc(Reshape);
	glutDisplayFunc(Start);
	glutIdleFunc(Update);
	glutMotionFunc(MouseMotion);
	glutKeyboardFunc(Keyboard);
	glutMainLoop();
}