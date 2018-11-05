#include <iostream>
#include <cstring>
#include <cstdlib>
#include <utility>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stack>
#include <set>
#include <map>
#include <cassert>
#define ll long long
#define pii pair<int, int>
#define A first
#define B second
#define mp make_pair
#define PI 3.14159265358979
#include "sdl_base.h"
using namespace std;
SDL_Texture *circle, *circleHull, *circleBad;
int n = 100, radius = 10;
void (*func)(), (*init)();
int W, H;
vector<pii> points;
int delay = 1, cnt = 0;
int cp(pii a, pii b, pii c)
{
    return (b.A - a.A) * (c.B - b.B) - (b.B - a.B) * (c.A - b.A);
}
int cp(int a, int b, int c)
{
    return cp(points[a], points[b], points[c]);
}
double distLinePoint(pii a, pii b, pii c)
{
    return fabs((b.B - a.B)*c.A - (b.A - a.A)*c.B + b.A*a.B - b.B*a.A) / sqrt(pow(b.B-a.B, 2) + pow(b.A-a.A, 2));
}
void genpoints()
{
    points.clear();
    double MAXD = 4*radius*radius;
    while(points.size() < n)
    {
        double angle = randf()*2*PI;
        double mag = sqrt(randf());
        pii nxt = mp(W/2 + mag*(W/2)*sin(angle), H/2 + mag*(H/2)*cos(angle));
        bool good = true;
        if(nxt.A<radius || nxt.B<radius || nxt.A+radius>W || nxt.B+radius>H)
            continue;
        for(auto &i: points)
        {
            if(pow(nxt.A-i.A, 2) + pow(nxt.B-i.B, 2) < MAXD)
            {
                good = false;
                break;
            }
        }
        if(good)
            points.push_back(nxt);
    }
}
namespace graham
{
    int lowest;
    vector<int> hull;
    set<int> bad;
    int cur;
    bool done, doneAdding;
    void init()
    {
        done = false;
        doneAdding = false;
        hull.clear();
        bad.clear();
        cur = 0;
        lowest = -1;
    }
    bool cmp(pii a, pii b)
    {
        return atan2(a.B - points[0].B, a.A - points[0].A) < atan2(b.B - points[0].B, b.A - points[0].A);
    }
    void go()
    {
        if(!done)
        {
            if(cnt < delay)
                cnt++;
            else
            {
                cnt = 0;
                for(int t=0; !done && t<max(1, 2-delay); t++){
                if(lowest == -1)
                {
                    lowest = 0;
                    for(int i=1; i<n; i++)
                        if(points[i].B > points[lowest].B)
                            lowest = i;
                    swap(points[lowest], points[0]);
                    sort(points.begin()+1, points.end(), cmp);
                    hull.push_back(0);
                    cur = 1;
                }
                if(hull.size()>2 && cp(hull[hull.size()-3], hull[hull.size()-2], hull.back())<0)
                {
                    bad.insert(*(hull.end()-2));
                    hull.erase(hull.end()-2);
                }
                else
                {
                    if(cur == n)
                        doneAdding = true;
                    else
                    {
                        hull.push_back(cur);
                        cur++;
                    }
                }
                if(cur==n && doneAdding)
                {
                    if(hull.size()>2 && cp(hull[hull.size()-2], hull.back(), hull[0])<0)
                    {
                        bad.insert(*(hull.end()-2));
                        hull.erase(hull.end()-2);
                    }
                    else done = true;
                }
                }
            }
        }
        renderClear(255, 255, 255);
        for(int i=0, hpos=0; i<points.size(); i++)
        {
            if(hpos<hull.size() && hull[hpos]==i)
            {
                hpos++;
                renderCopy(circleHull, points[i].A-radius, points[i].B-radius, radius*2, radius*2);
            }
            else if(bad.count(i))
                renderCopy(circleBad, points[i].A-radius, points[i].B-radius, radius*2, radius*2);
            else renderCopy(circle, points[i].A-radius, points[i].B-radius, radius*2, radius*2);
        }
        for(int i=1; i<hull.size(); i++)
            drawLine(points[hull[i]].A, points[hull[i]].B, points[hull[i-1]].A, points[hull[i-1]].B, 255, 0, 0);
        if(done)
            drawLine(points[hull[0]].A, points[hull[0]].B, points[hull.back()].A, points[hull.back()].B, 255, 0, 0);
    }
}
namespace quickhull
{//bugged. on the same set of points it sometimes works and sometimes doesn't, so it's inconsistent. probably memory issues?
    bool done, started;
    set<int> hull, bad;
    map<pii, int> lines;
    stack<pair<vector<int>, pii> > state;
    int removing;
    void init()
    {
        hull.clear();
        bad.clear();
        lines.clear();
        state = stack<pair<vector<int>, pii> >();
        done = false;
        started = false;
    }
    void go()
    {
        if(!done)
        {
            if(cnt < delay)
                cnt++;
            else
            {
                cnt = 0;
                for(int t=0; !done && t<max(1, 2-delay); t++){
                if(!started)
                {
                    sort(points.begin(), points.end());
                    started = true;
                    int minx = 0, maxx = 0;
                    for(int i=1; i<n; i++)
                    {
                        if(points[i].A < points[minx].A)
                            minx = i;
                        else if(points[i].A > points[maxx].A)
                            maxx = i;
                    }
                    vector<int> a, b;
                    for(int i=0; i<n; i++)
                    {
                        if(i!=minx && i!=maxx)
                        {
                            if(cp(minx, i, maxx) < 0)
                                a.push_back(i);
                            else b.push_back(i);
                        }
                    }
                    state.emplace(a, mp(minx, maxx));
                    state.emplace(b, mp(maxx, minx));
                    lines[mp(minx, maxx)] = 1;
                    hull.insert(minx);
                    hull.insert(maxx);
                    removing = -1;
                }
                else
                {
                    static pair<vector<int>, pii> x;
                    static int farthest;
                    if(removing == -1)
                    {
                        x = state.top();
                        state.pop();
                        farthest = x.A[0];
                        double bestd = 0;
                        for(int i=0; i<x.A.size(); i++)
                        {
                            double d = distLinePoint(points[x.B.A], points[x.B.B], points[x.A[i]]);
                            if(d > bestd)
                            {
                                bestd = d;
                                farthest = x.A[i];
                            }
                        }
                        hull.insert(farthest);
                        lines[mp(x.B.A, farthest)] = 1;
                        lines[mp(farthest, x.B.B)] = 1;
                        auto z = lines.find(mp(x.B.A, x.B.B));
                        z->B--;
                        if(z->B == 0)
                            lines.erase(z);
                        removing = 0;
                    }
                    else
                    {
                        static vector<int> a, b;
                        bool removedAll = false;
                        while(removing <= x.A.size())
                        {
                            if(removing == x.A.size())
                            {
                                removedAll = true;
                                break;
                            }
                            int v1 = cp(x.B.A, x.A[removing], farthest);
                            int v2 = cp(farthest, x.A[removing], x.B.B);
                            int v3 = cp(x.B.B, x.A[removing], x.B.A);
                            if(v1 < 0)
                                a.push_back(x.A[removing]);
                            else if(v2 < 0)
                                b.push_back(x.A[removing]);
                            else
                            {
                                bad.insert(x.A[removing]);
                                removing++;
                                break;
                            }
                            removing++;
                        }
                        if(removedAll)
                        {
                            removing = -1;
                            if(a.size() > 0)
                                state.emplace(a, mp(x.B.A, farthest));
                            if(b.size() > 0)
                                state.emplace(b, mp(farthest, x.B.B));
                            if(state.empty())
                                done = true;
                            a.clear();
                            b.clear();
                        }
                    }
                }
                }
            }
        }
        renderClear(255, 255, 255);
        for(int i=0; i<points.size(); i++)
        {
            if(hull.count(i))
                renderCopy(circleHull, points[i].A-radius, points[i].B-radius, radius*2, radius*2);
            else if(bad.count(i))
                renderCopy(circleBad, points[i].A-radius, points[i].B-radius, radius*2, radius*2);
            else renderCopy(circle, points[i].A-radius, points[i].B-radius, radius*2, radius*2);
        }
        setColor(0, 0, 0);
        for(auto &i: lines)
            drawLine(points[i.A.A].A, points[i.A.A].B, points[i.A.B].A, points[i.A.B].B, 255, 0, 0);
    }
}
namespace mchain
{
    bool started;
    vector<int> hull[2];
    set<int> bad;
    int cur;
    bool done, doneAdding;
    void init()
    {
        done = false;
        hull[0].clear();
        hull[1].clear();
        bad.clear();
        cur = 0;
        started = false;
        doneAdding = false;
    }
    void go()
    {
        if(!done)
        {
            if(cnt < delay)
                cnt++;
            else
            {
                cnt = 0;
                for(int t=0; !done && t<max(1, 2-delay); t++){
                if(!started)
                {
                    started = true;
                    sort(points.begin(), points.end());
                    hull[0].push_back(0);
                    hull[1].push_back(0);
                    cur = 1;
                }
                bool del = false;
                for(int h=0; h<2; h++)
                    if(hull[h].size()>2 && (!h ^ (cp(hull[h][hull[h].size()-3], hull[h][hull[h].size()-2], hull[h].back())>0)))
                    {
                        bad.insert(*(hull[h].end()-2));
                        hull[h].erase(hull[h].end()-2);
                        del = true;
                        break;
                    }
                if(!del)
                {
                    if(cur == n)
                        done = true;
                    else
                    {
                        hull[0].push_back(cur);
                        hull[1].push_back(cur);
                        cur++;
                    }
                }
                }
            }
        }
        renderClear(255, 255, 255);
        int hpos[2]{};
        for(int i=0; i<points.size(); i++)
        {
            bool onhull = false;
            for(int j=0; j<2; j++)
                if(hpos[j]<hull[j].size() && hull[j][hpos[j]]==i)
                {
                    hpos[j]++;
                    renderCopy(circleHull, points[i].A-radius, points[i].B-radius, radius*2, radius*2);
                    onhull = true;
                }
            if(!onhull)
            {
                if(bad.count(i))
                    renderCopy(circleBad, points[i].A-radius, points[i].B-radius, radius*2, radius*2);
                else renderCopy(circle, points[i].A-radius, points[i].B-radius, radius*2, radius*2);
            }
        }
        for(int h=0; h<2; h++)
            for(int i=1; i<hull[h].size(); i++)
                drawLine(points[hull[h][i]].A, points[hull[h][i]].B, points[hull[h][i-1]].A, points[hull[h][i-1]].B, 255, 0, 0);
    }
}
int main(int argc, char **argv)
{
    atexit(SDL_Quit);
    func = mchain::go;
    init = mchain::init;
    for(int i=1; i<argc; i++)
    {
        if(!strcmp("-n", argv[i]))
            n = atoi(argv[++i]);
        else if(!strcmp("-s", argv[i]))
            delay = atoi(argv[++i]);
        else if(!strcmp("-a", argv[i]))
        {
            if(!strcmp("graham", argv[++i]))
            {
                func = graham::go;
                init = graham::init;
            }
            else if(!strcmp("quickhull", argv[++i]))
            {
                func = quickhull::go;
                init = quickhull::init;
            }
            else if(!strcmp("mchain", argv[++i]))
            {
                func = mchain::go;
                init = mchain::init;
            }
            else println("Unsupported algorithm: " + (string)argv[i]);
        }
        else println("Unsupported argument: " + (string)argv[i]);
    }
    sdl_settings::load_config();
    initSDL();
    //create point textures
    circle = SDL_CreateTexture(getRenderer(), SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, radius*2, radius*2);
    SDL_SetTextureBlendMode(circle, SDL_BLENDMODE_BLEND);
    SDL_SetRenderTarget(getRenderer(), circle);
    fillCircle(radius, radius, radius, 0, 0, 0);
    //create hull point textures
    circleHull = SDL_CreateTexture(getRenderer(), SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, radius*2, radius*2);
    SDL_SetTextureBlendMode(circleHull, SDL_BLENDMODE_BLEND);
    SDL_SetRenderTarget(getRenderer(), circleHull);
    fillCircle(radius, radius, radius, 0, 0, 255);
    //create bad point textures
    circleBad = SDL_CreateTexture(getRenderer(), SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, radius*2, radius*2);
    SDL_SetTextureBlendMode(circleBad, SDL_BLENDMODE_BLEND);
    SDL_SetRenderTarget(getRenderer(), circleBad);
    fillCircle(radius, radius, radius, 200, 200, 200);
    //ok done with creating textures
    SDL_SetRenderTarget(getRenderer(), NULL);
    W = sdl_settings::WINDOW_W;
    H = sdl_settings::WINDOW_H;
    genpoints();
    init();
    while(true)
    {
        while(SDL_PollEvent(&input))
        {
            switch(input.type)
            {
            case SDL_QUIT:
                exit(0);
                break;
            case SDL_KEYDOWN:
                switch(input.key.keysym.sym)
                {
                case SDLK_MINUS:
                    delay = max(delay-1, -30);
                    break;
                case SDLK_EQUALS:
                    delay = min(delay+1, 100);
                    break;
                case SDLK_r:
                    genpoints();
                    init();
                    break;
                case SDLK_1:
                    func = graham::go;
                    init = graham::init;
                    init();
                    break;
                case SDLK_2:
                    func = quickhull::go;
                    init = quickhull::init;
                    init();
                    break;
                case SDLK_3:
                    func = mchain::go;
                    init = mchain::init;
                    init();
                    break;
                }
            }
        }
        func();
        if(func == graham::go)
            drawText("Graham Scan", 0, 0, H/30);
        else if(func == quickhull::go)
            drawText("Quickhull", 0, 0, H/30);
        else if(func == mchain::go)
            drawText("Monotone Chain", 0, 0, H/30);
        drawText("Delay:" + to_str(delay), 0, H/30, H/30);
        updateScreen();
    }
    return 0;
}
