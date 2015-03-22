#include <fstream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <cstdlib>

#include <GL/freeglut.h>

using namespace std;
const double eps = 1e-9;
const double INF = 1e9;
const double PI = 3.1416;
typedef struct poit{ double x,y,angle,angle2; int segmentIndex; } Point;
Point R, R2;
vector<Point> V, S, F, Light;
int N;
double Xmin, Xmax, Ymin, Ymax, Lmax;
bool* keyStates = new bool[256]; // Create an array of boolean values of length 256 (0-255)
bool* keySpecialStates = new bool[256]; // Create an array of boolean values of length 256 (0-255)

void ComputeVisibility(Point &R);

void keySpecial (int key, int x, int y) {
    keySpecialStates[key] = true;
}

void keySpecialUp (int key, int x, int y) {
    keySpecialStates[key] = false;
}

void keySpecialOperations(void) {
if (keySpecialStates[GLUT_KEY_LEFT]) { // If the left arrow key has been pressed
    R2.x += 0.1;
    R2.y += 0.1;
}
}
void keyOperations (void) {
    bool ok = 0;

    if (keyStates['a']) {
        R.x -= 0.1;
        ok = 1;
    }
    else if (keyStates['d']) {
        R.x += 0.1;
        ok = 1;
    }
    else if (keyStates['w']) {
        R.y += 0.1;
        ok = 1;
    }
    else if (keyStates['s']) {
        R.y -= 0.1;
        ok = 1;
    }

    if(ok)
    {
        ComputeVisibility(R);
    }
}

void keyPressed (unsigned char key, int x, int y) {
    keyStates[key] = true; // Set the state of the current key to pressed
}

void keyUp (unsigned char key, int x, int y) {
    keyStates[key] = false; // Set the state of the current key to not pressed
}

bool equal(double x, double y) { return fabs(x-y) < eps; };
double dist(const Point &P1, const Point &P2){ return (P1.x - P2.x) * (P1.x - P2.x) + (P1.y - P2.y) * (P1.y - P2.y); }
double det(Point &P, Point &R, Point &Q){ return (P.x * R.y) + (R.x * Q.y) + (Q.x * P.y) - (Q.x * R.y) - (P.x * Q.y) - (R.x * P.y); }

bool PolarAngleCmp(const Point &P, const Point &Q){

    //if(equal(P.angle, Q.angle))
    if(P.segmentIndex == Q.segmentIndex)
    {
        /*
        if((0 <= P.angle && P.angle <= 90) || (180 <= P.angle && P.angle <= 270))
        {
            return dist(R, P) < dist(R, Q);
        }
        else
        {
            return dist(R, Q) < dist(R, P);
        }

        return P.angle2 < Q.angle2;*/
        //if(P.segmentIndex < N && Q.segmentIndex < N) return P.segmentIndex < Q.segmentIndex;
        //else return P.segmentIndex < N;
        //if(270 <= P.angle && P.angle <= 360 && 0 <= R.angle && R.angle <= 90) return 1;
        //if(270 <= R.angle && R.angle <= 360 && 0 <= P.angle && P.angle <= 90) return 0;

        //return P.angle < Q.angle;
        return dist(P, V[P.segmentIndex]) < dist(Q, V[Q.segmentIndex]);
    }

    //return P.angle < Q.angle;
    return P.segmentIndex < Q.segmentIndex;
}

bool Intersects(Point &R1, Point &R2, Point &V1, Point &V2, Point &IPoint){
    double Vslope, Vcoef, Rslope, Rcoef;

    if(equal(R1.x, R2.x)) // Vertical ray
    {
        if(equal(V1.x, V2.x)) return 0;
        if(equal(R1.x, V1.x) || equal(R1.x, V2.x)) return 0; // segment endpoints
        if((R1.x > V1.x && R1.x > V2.x) || (R1.x < V1.x && R1.x < V2.x)) return 0;

        Vslope = (V1.y - V2.y) / (V1.x - V2.x);
        Vcoef = V1.y - Vslope * V1.x;

        double yy = Vslope * R1.x + Vcoef;
        if((R1.y < R2.y && R1.y > yy) || (R2.y < R1.y && R1.y < yy)) return 0;
        IPoint.x = R1.x;
        IPoint.y = yy;
        return 1;
    }

    Rslope = (R1.y - R2.y) / (R1.x - R2.x);
    Rcoef = R1.y - Rslope * R1.x;

    if(equal(V1.x, V2.x))
    {
        double yy = Rslope * V1.x  + Rcoef;
        if(equal(V1.y, yy) || equal(V2.y, yy)) return 0;
        if((yy > V1.y && yy > V2.y) || (yy < V1.y && yy < V2.y)) return 0;
        if(R1.x < R2.x && R1.x > V1.x) return 0;
        if(R1.x > R2.x && R1.x < V1.x) return 0;

        IPoint.x = V1.x;
        IPoint.y = yy;
        return 1;
    }

    Vslope = (V1.y - V2.y) / (V1.x - V2.x);
    Vcoef = V1.y - Vslope * V1.x;

    double xx = (Vcoef - Rcoef) / (Rslope - Vslope);

    if(equal(xx, V1.x) || equal(xx, V2.x)) return 0;
    if((xx > V1.x && xx > V2.x) || (xx < V1.x && xx < V2.x)) return 0;
    if(R1.x < R2.x && R1.x > xx) return 0;
    if(R1.x > R2.x && R1.x < xx) return 0;

    double yy = Vslope * xx + Vcoef;

    if((equal(xx,V1.x) && equal(yy,V1.y)) || (equal(xx,V2.x) && equal(yy,V2.y))) return 0;

    IPoint.x = xx;
    IPoint.y = Vslope * xx + Vcoef;
    return 1;
}

void ComputeVisibility(Point &R){
    int i,j;
    Point IPoint, PtoAdd;
    double distance;
    double dmin;
    int segmentIndex;

    S.clear();
    Light.clear();

    for(i=0;i<N-1;++i)
    {
        dmin = INF;
        PtoAdd = V[i];
        segmentIndex = i;

        for(j=0;j<N-1;++j)
            if(j != i-1 && j != i)
            {
                if(Intersects(R, V[i], V[j], V[j+1], IPoint))
                {
                    distance = dist(R, IPoint);
                    if(distance < dmin)
                    {
                        dmin = distance;
                        PtoAdd = IPoint;
                        segmentIndex = j;
                    }
                }
            }

        if(dist(R, V[i]) < dist(R, PtoAdd))
        {
            Point aux = V[i];
            aux.segmentIndex = i;
            S.push_back(aux);
        }

        Point aux = (i == 0) ? V[N-1] : V[i-1];

        double d1 = det(V[i], PtoAdd, aux), d2 = det(V[i], PtoAdd, V[i+1]);

        if((d1 < -eps && d2 > eps) || (d1 > eps && d2 < -eps)) continue;

        PtoAdd.segmentIndex = segmentIndex;
        S.push_back(PtoAdd);
    }
    unsigned int Size = S.size();

    //for(i=0;i<Size;++i)
    //  cout << S[i].x << ' ' << S[i].y << '\n';
    //cout << '\n';

    for(i=0;i<Size;++i)
    {
        if(equal(R.x, S[i].x))
        {
            if(S[i].y > R.x)
                S[i].angle = 90;
            else
                S[i].angle = 270;
            continue;
        }

        S[i].angle = 180 * atan((S[i].y - R.y) / (S[i].x - R.x)) / PI;

        if(S[i].x < R.x) S[i].angle += 180;

        if(S[i].x > R.x && S[i].y < R.y) S[i].angle += 360;
    }

    //for(i=0;i<Size;++i)
    //  cout << S[i].x << ' ' << S[i].y << ' ' << S[i].angle << '\n';

    double MinAngle = 380, MaxAngle = -1;
    for(i=0;i<Size;++i)
    {
        if(S[i].angle < MinAngle) MinAngle = S[i].angle;
        if(S[i].angle > MaxAngle) MaxAngle = S[i].angle;
    }

    Point PP;

    for(i=0;i<Size;++i)
    {
    /*
        if(equal(S[i].angle, MinAngle))
        {
            PP = S[i];
            PP.angle += 360;
            S.push_back(PP);
        }

        if(equal(S[i].angle, MaxAngle))
        {
            PP = S[i];
            PP.angle -= 360;
            S.push_back(PP);
        }
        */
    }


    sort(S.begin(), S.end(), PolarAngleCmp);

    //for(i=0;i<Size-1;++i)
    //    if(equal(S[i].angle, S[i+1].angle) && S[i].angle2 > S[i+1].angle2)
     //       swap(S[i], S[i+1]);

    //for(i=0;i<Size;++i)
    //  cout << S[i].x << ' ' << S[i].y << ' ' << S[i].segmentIndex <<  ' ' << S[i].angle << '\n';

    Size = S.size();
    for(i=0;i<Size;++i)
    {
        PP.x = 1.9 * ((S[i].x - Xmin) / Lmax) - 0.95;
        PP.y = 1.9 * ((S[i].y - Ymin) / Lmax) - 0.95;
        Light.push_back(PP);
    }

    R2.x = 1.9 * ((R.x - Xmin) / Lmax) - 0.95;
    R2.y = 1.9 * ((R.y - Xmin) / Lmax) - 0.95;

}


static void RenderSceneCB()
{
    keyOperations();

    glClearColor(0.1f, 0.0f, 0.0f, 0.1f);
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity(); // Load the Identity Matrix to reset our drawing locations

    glColor3f(1.0f, 1.0f, 1.0f);

    glBegin(GL_LINES);
    for(int i=0;i<N-1;++i)
    {
        glVertex2d(F[i].x, F[i].y);
        glVertex2d(F[i+1].x, F[i+1].y);

    }
    glEnd();


    unsigned int Size = Light.size();


    glBegin(GL_LINES);
    for(int i=0;i<Size - 1;++i)
    {
        glColor3f(0.0f, 1.0f, 0.0f);
        //glColor3f(1.0 * (RAND_MAX - rand()) / RAND_MAX, 1.0 * (RAND_MAX - rand()) / RAND_MAX, 1.0 * (RAND_MAX - rand()) / RAND_MAX);
        glVertex2d(Light[i].x, Light[i].y);
        glVertex2d(Light[i+1].x, Light[i+1].y);
    }
        glVertex2d(Light[Size-1].x, Light[Size-1].y);
        glVertex2d(Light[0].x, Light[0].y);
    glEnd();

    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_QUADS);
        glVertex2d(R2.x, R2.y);
        glVertex2d(R2.x+0.01, R2.y);
        glVertex2d(R2.x+0.01, R2.y+0.01);
        glVertex2d(R2.x, R2.y+0.01);
    glEnd();

/*
    glBegin(GL_QUADS);
        glVertex2d(-0.5, -0.5);
        glVertex2d(-0.5, 0.5);
        glVertex2d(0.5, 0.5);
        glVertex2d(0.5, -0.5);
    glEnd();*/

    glutSwapBuffers();
    glutPostRedisplay();
}

static void InitializeGlutCallbacks()
{
    glutDisplayFunc(RenderSceneCB);
    glutKeyboardFunc(keyPressed); // Tell GLUT to use the method "keyPressed" for key presses
    glutKeyboardUpFunc(keyUp); // Tell GLUT to use the method "keyUp" for key up events
    glutSpecialFunc(keySpecial); // Tell GLUT to use the method "keySpecial" for special key presses
    glutSpecialUpFunc(keySpecialUp); // Tell GLUT to use the method "keySpecialUp" for special up key events
}

int main(int argc, char** argv){
    ifstream cin("Visibility.in");
    ofstream cout("Visibility.out");
    int i,j;
    cin >> R.x >> R.y;

    cin >> N;

    Point aux;
    for(i=1;i<=N;++i)
    {
        cin >> aux.x >> aux.y;
        V.push_back(aux);
    }

    ++N;
    V.push_back(*V.begin());

    Xmin = Ymin = INF;
    Xmax = Ymax = -INF;
    for(i=0;i<N;++i)
    {
        if(V[i].x < Xmin) Xmin = V[i].x;
        if(V[i].x > Xmax) Xmax = V[i].x;
        if(V[i].y < Ymin) Ymin = V[i].y;
        if(V[i].y > Ymax) Ymax = V[i].y;
    }

    Lmax = max(Xmax - Xmin, Ymax - Ymin);

    Point PP;
    for(i=0;i<N;++i)
    {
        PP.x = 1.9 * ((V[i].x - Xmin) / Lmax) - 0.95;
        PP.y = 1.9 * ((V[i].y - Ymin) / Lmax) - 0.95;
        F.push_back(PP);
    }

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);
    glutInitWindowSize(1024, 768);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Visibility Poligon");
    InitializeGlutCallbacks();

    ComputeVisibility(R);

    glutMainLoop();

    return 0;
}
