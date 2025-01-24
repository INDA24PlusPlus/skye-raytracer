#include<stdio.h>
#include<GL/glut.h>
#include<math.h> 
#include<experimental/simd>
namespace stdx=std::experimental;
#define pi 3.142857 
#define width 1000
#define height 800
typedef enum {
	OBJ_SPHERE,
	OBJ_TRI,
	OBJ_PLANE
} ObjectType;

typedef struct {
	float* Midpoint;
	float* Radius;
} Sphere;

typedef struct {
	float* Normal;
	float* Distance;
} Plane;

typedef struct {
	float* P1;
	float* P2;
	float* P3;
} Tri;

typedef union {
	Sphere* sph;
	Plane* pln;
	Tri* tri;
} ObjectWrapper;

typedef struct {
	float* rgb;
	float* alpha;
} Material;

typedef struct {
	ObjectType oType;
	ObjectWrapper object;
	Material* oMaterial;
} Object;

float* instanceiateVector(float x, float y, float z){
	float* A=(float*)malloc(sizeof(float)*3);
	A[0]=x;
	A[1]=y;
	A[2]=z;
	return A;
}
float* cloneVector(float* A){
	return instanceiateVector(A[0], A[1], A[2]);
}

void printVector(float* A){
	printf("(%f,%f,%f)\n",A[0],A[1],A[2]);
}

void vectorMul(float* A, float b, float* out){
	out[0]=b*A[0];
	out[1]=b*A[1];
	out[2]=b*A[2];
}

void vectorAdd(float* A, float* B, float* out){
	out[0]=A[0]+B[0];
	out[1]=A[1]+B[1];
	out[2]=A[2]+B[2];
}

void vectorSub(float* A, float* B, float* out){
	out[0]=A[0]-B[0];
	out[1]=A[1]-B[1];
	out[2]=A[2]-B[2];
}

void crossProduct(float* A, float* B, float* out){
	out[0]=A[1]*B[2]-A[2]*B[1];
	out[1]=A[2]*B[0]-A[0]*B[2];
	out[2]=A[0]*B[1]-A[1]*B[0];
}

float dotProduct(float* A, float* B){
	return (A[0]*B[0]+A[1]*B[1]+A[2]*B[2]);
}

float vectorLength(float* A){
	return sqrtf(dotProduct(A, A));
}

void vectorNormalise(float* A, float* out){
	float o=vectorLength(A);
	if(o==0){
		printf("Cannot normalise vector of length 0\n");
		exit(3);
	}
	out[0]=A[0]/(o);
	out[1]=A[1]/(o);
	out[2]=A[2]/(o);
}

int vectorEq(float* A, float* B){
	return A[0]==B[0]&&A[1]==B[1]&&A[2]==B[2];
}

typedef struct {
	float* Pos;
	float* FOV;
	float* AspectRatio;
	float* fScale;
} Camera;

typedef struct {
	float* Origin;
	float* Direction;
} Ray;
typedef struct {
	Object** Objects;
	int* cObj;
	float* Light;
	Camera* Cam;
} World;
Camera* genCamera(float* P, float fov, float ar){
	Camera* cam=(Camera*)malloc(sizeof(Camera));
	cam->FOV=(float*)malloc(sizeof(float));
	*(cam->FOV)=fov;
	cam->AspectRatio=(float*)malloc(sizeof(float));
	*(cam->AspectRatio)=ar;
	cam->Pos=P;
	cam->fScale=(float*)malloc(sizeof(float));
	*(cam->fScale)=tanf(*(cam->FOV)/2.0);
	return cam;
}

World* getTestWorld(){
	World* W=(World*)malloc(sizeof(World));
	W->cObj=(int*)malloc(sizeof(int));
	*(W->cObj)=2;
	W->Objects=(Object**)malloc(sizeof(Object*)*(*W->cObj+1));
	W->Objects[0]=(Object*)malloc(sizeof(Object));
	// Sphere
	W->Objects[0]->object.sph=(Sphere*)malloc(sizeof(Sphere));
	W->Objects[0]->object.sph->Midpoint=instanceiateVector(-2, 1, 5);
	float* a=(float*)malloc(sizeof(float));
	*a=2;
	W->Objects[0]->object.sph->Radius=a;
	W->Objects[0]->oType=OBJ_SPHERE;
	W->Objects[0]->oMaterial=(Material*)malloc(sizeof(Material));
	a=(float*)malloc(sizeof(float));
	*a=1;
	W->Objects[0]->oMaterial->rgb=instanceiateVector(1, 0, 0);
	W->Objects[0]->oMaterial->alpha=a;
	W->Objects[1]=(Object*)malloc(sizeof(Object));
	// Ground
	W->Objects[1]->object.pln=(Plane*)malloc(sizeof(Plane));
	W->Objects[1]->object.pln->Normal=instanceiateVector(0, 1, 0);
	a=(float*)malloc(sizeof(float));
	*a=1;
	W->Objects[1]->object.pln->Distance=a;
	W->Objects[1]->oType=OBJ_PLANE;
	W->Objects[1]->oMaterial=(Material*)malloc(sizeof(Material));
	a=(float*)malloc(sizeof(float));
	*a=1;
	W->Objects[1]->oMaterial->rgb=instanceiateVector(0, 1, 0);
	W->Objects[1]->oMaterial->alpha=a;

	W->Light=instanceiateVector(0, -7, 7);
	W->Cam=genCamera(instanceiateVector(0, 0, 0), pi/2.0, (float)width/(float)height);
	return W;

}
World* world;

Ray* rayFromUV(float u, float v){
	float vX=(*world->Cam->fScale)*(*world->Cam->AspectRatio)*(2*u-1);
	float vY=(*world->Cam->fScale)*(1-2*v);
	float* D1=instanceiateVector(vX, vY, 1);
	vectorNormalise(D1, D1);
	Ray* r=(Ray*)malloc(sizeof(Ray));
	r->Origin=cloneVector(world->Cam->Pos);
	r->Direction=D1;
	return r;
}
float intersectSphere(Ray* r, Sphere* s){
	float* V=instanceiateVector(0, 0, 0);
	vectorSub(r->Origin, s->Midpoint, V);
	float a=dotProduct(r->Direction, r->Direction);
	if(a==0){free(V);return -2;}
	float b=2*dotProduct(V, r->Direction);
	float c=dotProduct(V, V)-(*s->Radius)*(*s->Radius);
	float rt1=b*b-4*a*c;
	if(rt1==0){free(V);return -2;}
	float rt=sqrtf(rt1);
	float t1=(-b-rt)/(2*a);
	if(t1<=0) t1=(-b+rt)/(2*a);
	free(V);
	return t1;
}

float intersectPlane(Ray* r, Plane* p){
	if(dotProduct(r->Direction,p->Normal)==0){return -2;}
	return ((*p->Distance)-dotProduct(r->Origin, p->Normal))/dotProduct(r->Direction, p->Normal);
}

float intersectTri(Ray* r, Tri* t){
	float* t1=instanceiateVector(0, 0, 0);
	float* t2=cloneVector(t1);
	vectorSub(t->P2, t->P1, t1);
	vectorSub(t->P3, t->P1, t2);
	float* N=cloneVector(t1);
	crossProduct(t1, t2, N);
	vectorNormalise(N, N);
	float d=dotProduct(t->P1, N);
	float tV=(d-dotProduct(r->Origin, N))/dotProduct(r->Direction, N);
	free(N);
	free(t1);
	free(t2);
	return tV;
}

float intersectObject(Ray* r, Object* o){
	if(o->oType==OBJ_SPHERE){
		return intersectSphere(r,o->object.sph);
	}
	if(o->oType==OBJ_PLANE){
		return intersectPlane(r, o->object.pln);
	}
	printf("aaaaaaaaaaaaaaaaa\n");
	exit(1);
}

Object* firstHit(Ray* r){
	Object* obj=NULL;
	float mT=-1;
	for(int i=0; i<*world->cObj; i++){
		float T=intersectObject(r, world->Objects[i]);
		if(T>0&&(mT==-1||T<mT)){
			mT=T;
			obj=world->Objects[i];
		}
	}
	return obj;
}
void clampColour(float* C){
	if(C[0]>1) C[0]=1;
	if(C[1]>1) C[1]=1;
	if(C[2]>1) C[2]=1;
}
void absColour(float* C){
	if(C[0]<0) C[0]=-C[0];
	if(C[1]<0) C[1]=-C[1];
	if(C[2]<0) C[2]=-C[2];

}

float* colourFromUV(float u, float v){
Ray* r=rayFromUV(u,v);
Object* obj=firstHit(r);
if(obj==NULL){
	free(r);
	return instanceiateVector(0.3,0.6,0.8);
}
float* colour=cloneVector(obj->oMaterial->rgb);
float t=intersectObject(r, obj);
float* V=instanceiateVector(0, 0, 0);
float* V2=cloneVector(V);
vectorMul(r->Direction,t,V);
vectorAdd(r->Origin,V,V2);
vectorSub(world->Light,V2,V);
vectorNormalise(V,V);
Ray* sR=(Ray*)malloc(sizeof(Ray));
sR->Direction=V;
sR->Origin=V2;
Object* obj2=firstHit(sR);
if(obj2!=NULL){
	vectorMul(colour, 0.3, colour);
}
else{
	float* V3=cloneVector(V2);
	float* V4=cloneVector(V2);
	
	vectorMul(V, -1, V3);
	if(obj->oType==OBJ_SPHERE){

			vectorSub(V2, obj->object.sph->Midpoint, V4);
			//vectorNormalise(V4, V4);
			vectorMul(colour, dotProduct(V,V4)*1.2,colour);
			clampColour(colour);
		}
		if(obj->oType==OBJ_PLANE){
			vectorMul(colour, dotProduct(V3,obj->object.pln->Normal)*2,colour);
			clampColour(colour);

		}
		free(V3);
		free(V4);
	}
	free(sR);
	free(r);
	free(V);
	free(V2);
	absColour(colour);
	return colour;
}

float* colourFromRay(Ray* r, int depth){
	Object* obj=firstHit(r);
	if(obj==NULL){
		free(r);
		return instanceiateVector(0.1,0,0.8);
	}
	float* colour=cloneVector(obj->oMaterial->rgb);
	float t=intersectObject(r, obj);
	float* V=instanceiateVector(0, 0, 0);
	float* V2=cloneVector(V);
	vectorMul(r->Direction,t,V);
	vectorAdd(r->Origin,V,V2);
	vectorSub(world->Light,V2,V);
	vectorNormalise(V,V);
	Ray* sR=(Ray*)malloc(sizeof(Ray));
	sR->Direction=V;
	sR->Origin=V2;
	Object* obj2=firstHit(sR);
	if(obj2!=NULL){
		vectorMul(colour, 0.5, colour);
	}
	else{
		float* V3=cloneVector(V2);
		float* V4=cloneVector(V2);
		
		vectorMul(V2, -1, V3);
		if(obj->oType==OBJ_SPHERE){

			vectorSub(V2, obj->object.sph->Midpoint, V4);
			vectorNormalise(V4, V4);
			vectorMul(colour, dotProduct(V3,V4),colour);
			clampColour(colour);
		}
		free(V3);
		free(V4);
	}
	free(sR);
	free(r);
	free(V);
	free(V2);
	return colour;
}

void freeCam(Camera* C){
	free(C->Pos);
	free(C->FOV);
	free(C->AspectRatio);
	free(C->fScale);
	free(C);
}

void myInit (void) 
{ 
	glClearColor(0.0, 0.0, 0.0, 1.0); 
	
	glColor3f(0.0, 1.0, 0.0); 
	
	glPointSize(1.0); 
	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity(); 
	
	gluOrtho2D(0,width,0,height); 
} 

void display (void) 
{ 
	glClear(GL_COLOR_BUFFER_BIT); 
	glBegin(GL_POINTS); 
	for(int w=0; w<width;w++){
		for(int h=0; h<height;h++){
			float u=((float)w)/((float)width);
			float v=((float)h)/((float)height);
			float* c=colourFromUV(u,v);
			glColor3f(c[0], c[1], c[2]);
			free(c);
			glVertex2i(w, h); 
			
		}
	}
	glEnd(); 
	glFlush(); 
} 

int main (int argc, char** argv) 
{
	world=getTestWorld();
	float* A=instanceiateVector(0.3, 1.5, -3.1);

	float* B=instanceiateVector(0, -2.5, 1.3);
	float* C=instanceiateVector(0.3, 1.5, -3.1);
	float* D=instanceiateVector(0.3, 1.5, -3.1);
	A[0]=0.3;
	A[1]=1.5;
	A[2]=-3.1;
	printVector(A);
	printVector(B);
	crossProduct(A,B,C);
	printVector(C);
	glutInit(&argc, argv); 
	glutInitDisplayMode(GLUT_SINGLE| GLUT_RGBA); 
	
	glutInitWindowSize(width, height); 
	glutInitWindowPosition(0, 0); 
	
	// Giving name to window 
	glutCreateWindow("Circle Drawing"); 
	myInit(); 
	
	glutDisplayFunc(display); 
	glutMainLoop(); 
	free(A);
	free(B);
	free(C);
	free(D);
} 


