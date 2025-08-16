#ifndef PUFFERBALL_H
#define PUFFERBALL_H

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "raylib.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>   

#define MAX_BALLS           10
#define BALL_SPEED_MIN      220.0f
#define BALL_SPEED_MAX      360.0f
#define BALL_RADIUS         10

#define SPAWN_ANGLE_JITTER  (float)(M_PI / 6) // +-30 degrees in radians
#define CORNER_RADIUS       75.0f
#define SPAWN_MARGIN        1.0f
#define BUMPER_INFLUENCE    0.50f //How much bump velocity biases reflection
#define SEPARATION_EPSILON  0.75f //Sticking Avoidance

#define AMP_COOLDOWN_SEC    4.0f
#define WAVE_DURATION_SEC   0.40f
#define WAVE_SPEED_PX_S     900.0f
#define WAVE_RANGE_SCALE    1.35f
#define WAVE_SPEED_BOOST    1.35f

enum { ARENA_PX = 800 };
enum { HUD_TOP_PX = 75 };
enum { NUM_AGENTS = 4 };

#ifndef M_PI
#define M_PI                3.14159265
#endif

static const int   kStartingLives   = 15;
static const float kFixedDt         = 1.0f/60.0f;  // deterministic sim step
static const float kSpawnCooldown   = 0.75f;       // seconds with no balls before auto-spawn
static const float kSpawnPeriod     = 5.0f;        // spawn a new ball every 5s
static const float kBumperRadius    = 45.0f;       // pick the size you want    
static const float kSpawnOffset     = CORNER_RADIUS + SPAWN_MARGIN;

typedef struct {
    float perf;
    float score;
    float episode_return;
    float episode_length;
    float n;
} Log;

typedef struct { int x, y ,w, h; } RectI;

typedef struct {
    int win_w, win_h, world_size;
    RectI hud_top_px, arena_px;
    float world_to_px;
    Camera2D cam;
} Client;


typedef struct { float x, y; } Vector2f;

// Construct & ops
static inline Vector2f v2(float x, float y)          { return (Vector2f){x,y}; }
static inline Vector2f v2_add(Vector2f a, Vector2f b){ return v2(a.x+b.x, a.y+b.y); }
static inline Vector2f v2_sub(Vector2f a, Vector2f b){ return v2(a.x-b.x, a.y-b.y); }
static inline Vector2f v2_mul(Vector2f v, float s)   { return v2(v.x*s, v.y*s); }
static inline float    v2_len_sq(Vector2f v)         { return v.x*v.x + v.y*v.y; }
static inline float    v2_len(Vector2f v)            { return sqrtf(v2_len_sq(v)); }
static inline Vector2f v2_norm(Vector2f v)           { float L=v2_len(v); return (L>0)? v2(v.x/L, v.y/L) : v2(0,0); }
static inline float    v2_dot(Vector2f a, Vector2f b){ return a.x*b.x + a.y*b.y; }
static inline Vector2f v2_rot(Vector2f v, float ang) { float c=cosf(ang), s=sinf(ang); return v2(c*v.x - s*v.y, s*v.x + c*v.y); }
static inline float    v2_axis(Vector2f v, int axis)       { return axis ? v.y : v.x; }
static inline float    v2_axis_orth(Vector2f v, int axis)  { return axis ? v.x : v.y; }


typedef struct {
    Vector2f        position;   
    float           length;        
    float           height_at_center;        
    float           rotation;   
    Vector2f        velocity; //Influence from bumper motion
    unsigned char   state;
    float           state_timer;
} Bumper;

typedef struct {
    int     agent;
    int     axis;
    int     dir;
    float   boundary;
    float   other_min;
    float   other_max;
} GoalSpec;

typedef struct {
    Vector2f        position;
    Vector2f        velocity;
    int             lives;
    bool            eliminated;
    Bumper          bumper;
    bool            wave_active;
    float           wave_time;
    float           wave_cooldown;
    Bumper          wave_bumper;
} Agent; 

typedef struct {
    Vector2f    position;
    Vector2f    velocity;
    bool        active;
} Ball;


typedef struct {
    Log             log;
    Agent*          agents;
    float*          observations;
    int*            actions;
    float*          rewards;
    unsigned char*  terminals;
    
    Agent           agents_state[NUM_AGENTS];
    Ball            balls[MAX_BALLS];
    float           time_since_last_spawn;
    bool            game_over;
    int             winner_id;

    int             arena_x;
    int             arena_y;
    int             arena_right;
    int             arena_bottom;
    Vector2f        arena_center;

} pufferball;

// ---- Forward declarations ----

// core lifecycle
void init(pufferball* env);
void c_reset(pufferball* env);
void c_step(pufferball* env);
void c_render(pufferball* env, Client* c);
void c_close(pufferball* env, Client* c);

// sim pieces
void spawn_ball(pufferball* env);
void update_ball_positions(pufferball* env, float dt);
void handle_ball_collisions(pufferball* env);
void update_agent_states(pufferball* env, float dt);

// collisions with bumpers
bool check_ball_bumper_collision(Ball* ball, Bumper* bumper);
void reflect_ball(Ball* ball, const Bumper* bumper);

// rules/scoring
void check_goal_hit(pufferball* env);
void eliminate_agent(pufferball* env, int agent_id);
void check_game_over(pufferball* env);

// observations (stubbed)
void compute_observations(pufferball* env);



//Helper Functions

//UI Layout (I am not fully convinved this separation of "World Units and Coordinate Geometery makes sense")
static inline Client* client_create(int world_size) {
    InitWindow(ARENA_PX, ARENA_PX + HUD_TOP_PX, "Pufferball");
    SetTargetFPS(60);
    if (!IsWindowReady()) return NULL;
    Client* c = calloc(1, sizeof(Client));
    c->win_w = ARENA_PX; c->win_h = ARENA_PX + HUD_TOP_PX; c->world_size = world_size;
    c->hud_top_px = (RectI){0,0, ARENA_PX, HUD_TOP_PX};
    c->arena_px   = (RectI){0,HUD_TOP_PX, ARENA_PX, ARENA_PX};
    c->world_to_px = (float)ARENA_PX / (float)world_size;
    c->cam = (Camera2D){
        .target = (Vector2){ world_size*0.5f, world_size*0.5f },
        .offset = (Vector2){ c->arena_px.x + c->arena_px.w*0.5f, c->arena_px.y + c->arena_px.h*0.5f },
        .rotation = 0.0f,
        .zoom = c->world_to_px
    };
    return c;
}

static inline void client_destroy(Client* c){ if (c){ CloseWindow(); free(c);} }

static inline float rnd_range(float a, float b){
    return a + ((float)rand() / (float)RAND_MAX) * (b - a);
}

static int alloc_ball_slot(pufferball* env) {
    for (int i = 0; i < MAX_BALLS; ++i) if (!env->balls[i].active) return i;
    return -1;
}

static inline float wrap_pi(float a){
    while (a <= -M_PI) a += 2.0f * (float)M_PI;
    while (a > M_PI) a -= 2.0f * (float)M_PI;
    return a;
}

static inline Vector2f outward_normal_from_rotation(float rot){
    float c = cosf(rot), s = sinf(rot);
    return (Vector2f) { -s, c };
}

//Goal Helpers
static inline bool betweenf(float v, float a, float b) { return v >= a && v <= b; }
static inline float goal_min_x(const pufferball* env) { return env->arena_x + CORNER_RADIUS; }
static inline float goal_max_x(const pufferball* env) { return env->arena_right - CORNER_RADIUS; }
static inline float goal_min_y(const pufferball* env) { return env->arena_y + CORNER_RADIUS; }
static inline float goal_max_y(const pufferball* env) { return env->arena_bottom - CORNER_RADIUS; }

//Corner Circle Helpers
static inline void reflect_circle(Ball* b, float cx, float cy) {
    float nx = b->position.x - cx;
    float ny = b->position.y - cy;
    float L = sqrtf(nx*nx + ny*ny);
    if (L == 0) return;
    nx /= L; ny /= L;                     // outward normal
    float vn = b->velocity.x*nx + b->velocity.y*ny;
    if (vn >= 0) return;                  // moving away, no bounce
    // reflect
    b->velocity.x -= 2.0f * vn * nx;
    b->velocity.y -= 2.0f * vn * ny;
    // tiny separation
    b->position.x += nx * 0.5f;
    b->position.y += ny * 0.5f;
}

static inline bool handle_corner_reflectors(pufferball* env, Ball* b) {
    const float r2 = (CORNER_RADIUS + BALL_RADIUS) * (CORNER_RADIUS + BALL_RADIUS);

    // four corners (arena rect must be precomputed in env)
    float cx[4] = { (float)env->arena_x, (float)env->arena_right,
                    (float)env->arena_right, (float)env->arena_x };
    float cy[4] = { (float)env->arena_y, (float)env->arena_y,
                    (float)env->arena_bottom, (float)env->arena_bottom };

    for (int k = 0; k < 4; ++k) {
        float dx = b->position.x - cx[k];
        float dy = b->position.y - cy[k];
        float d2 = dx*dx + dy*dy;

        if (d2 <= r2) {
            reflect_circle(b, cx[k], cy[k]);
            return true;  // corner hit and reflection done
        }
    }

    return false; // no corners hit
}

static inline void despawn_balls(pufferball* env) {
    for (int i = 0; i < MAX_BALLS; ++i) env->balls[i].active = false;
}

// Place 4 agents centered on each side, facing inward
static void init_agents_on_walls(pufferball* env) {
    const int   S  = env->arena_x;
    const float cx = S * 0.5f;

    // Top (agent 0): outward normal points up, so use rotation = PI
    Agent* top = &env->agents_state[0];
    *top = (Agent){0};
    top->position                   = (Vector2f){ cx, 0.0f };
    top->lives                      = kStartingLives;
    top->eliminated                 = false;
    top->bumper.position            = top->position;
    top->bumper.length              = 1;
    top->bumper.height_at_center    = 1;
    top->bumper.rotation            = (float)M_PI;


    // Right (agent 1): outward +X, rotation = -PI/2
    Agent* right = &env->agents_state[1];
    *right = (Agent){0};
    right->position                 = (Vector2f){ (float)S, cx };
    right->lives                    = kStartingLives;
    right->eliminated               = false;
    right->bumper.position          = right->position;
    right->bumper.length            = 1;
    right->bumper.height_at_center  = 1;
    right->bumper.rotation          = -(float)M_PI/2.0f;

    // Bottom (agent 2): outward +Y, rotation = 0
    Agent* bottom = &env->agents_state[2];
    *bottom = (Agent){0};
    bottom->position                = (Vector2f){ cx, (float)S };
    bottom->lives                   = kStartingLives;
    bottom->eliminated              = false;
    bottom->bumper.position         = bottom->position;
    bottom->bumper.length           = 1;
    bottom->bumper.height_at_center = 1;
    bottom->bumper.rotation         = 0.0f;

    // Left (agent 3): outward -X, rotation = +PI/2
    Agent* left = &env->agents_state[3];
    *left = (Agent){0};
    left->position                  = (Vector2f){ 0.0f, cx };
    left->lives                     = kStartingLives;
    left->eliminated                = false;
    left->bumper.position           = left->position;
    left->bumper.length             = 1;
    left->bumper.height_at_center   = 1;
    left->bumper.rotation           = (float)M_PI/2.0f;
}

void init(pufferball* env) {
    
    srand((unsigned)time(NULL));

    env->arena_x                = 0;
    env->arena_y                = 0;
    env->arena_right            = ARENA_PX;
    env->arena_bottom           = ARENA_PX;
    env->arena_center           = (Vector2f){ ARENA_PX * 0.5f, ARENA_PX * 0.5f };
    env->log                    = (Log){0};
    env->time_since_last_spawn  = 0.0f;
    env->game_over              = false;
    env->winner_id              = -1;
    
    despawn_balls(env);
    init_agents_on_walls(env);
    spawn_ball(env);
}

void c_reset(pufferball* env) {

    env->log                    = (Log){0};
    env->game_over              = false;
    env->winner_id              = -1;
    env->time_since_last_spawn  = 0.0f;

    init_agents_on_walls(env);
    despawn_balls(env);
    spawn_ball(env);
}

void c_step(pufferball* env) {
    
    if (env->game_over) return;

    const float dt = kFixedDt;

    update_agent_states(env, dt);
    update_ball_positions(env, dt);
    handle_ball_collisions(env);
    check_goal_hit(env);

    if (env->game_over) return;

    bool any_active = false;
    for (int i = 0; i < MAX_BALLS; ++i) {
        if (env->balls[i].active) { any_active = true; break; }
    }
    if (!any_active) {
        spawn_ball(env);
        env->time_since_last_spawn = 0.0f;
    } else {
        env->time_since_last_spawn += dt;
        if (env->time_since_last_spawn >= kSpawnPeriod) {
            env->time_since_last_spawn = 0.0f;
            spawn_ball(env);
        }
    }

    // compute_observations(env); // keep stubbed if you want
}


// Simple world drawer used by c_render (balls + goal mouths + paddles + corner circles)
static void draw_world_2d(const void* ptr) {
    const pufferball* env = (const pufferball*)ptr;

    // Colors per agent: Top, Right, Bottom, Left
    const Color AGENT_COLORS[4] = {
        (Color){255,  64,  64, 255}, // Top: red-ish
        (Color){  0, 255, 255, 255}, // Right: cyan
        PINK,                        // Bottom: pink (raylib color)
        YELLOW                      // Left: yellow (raylib color)
    };

    // Goal mouths (for visual clarity)
    const float gx_min = goal_min_x(env), gx_max = goal_max_x(env);
    const float gy_min = goal_min_y(env), gy_max = goal_max_y(env);

    // Top goal
    DrawLine((int)gx_min, env->arena_y, (int)gx_max, env->arena_y, (Color){80,160,255,255});
    // Right goal
    DrawLine(env->arena_right, (int)gy_min, env->arena_right, (int)gy_max, (Color){80,160,255,255});
    // Bottom goal
    DrawLine((int)gx_min, env->arena_bottom, (int)gx_max, env->arena_bottom, (Color){80,160,255,255});
    // Left goal
    DrawLine(env->arena_x, (int)gy_min, env->arena_x, (int)gy_max, (Color){80,160,255,255});

    // Corner reflectors (visualize)
    DrawCircleLines(env->arena_x,      env->arena_y,      (int)CORNER_RADIUS, GRAY);
    DrawCircleLines(env->arena_right,  env->arena_y,      (int)CORNER_RADIUS, GRAY);
    DrawCircleLines(env->arena_right,  env->arena_bottom, (int)CORNER_RADIUS, GRAY);
    DrawCircleLines(env->arena_x,      env->arena_bottom, (int)CORNER_RADIUS, GRAY);

    // Balls
    for (int i = 0; i < MAX_BALLS; ++i) {
        const Ball* b = &env->balls[i];
        if (!b->active) continue;
        DrawCircleV((Vector2){ b->position.x, b->position.y }, BALL_RADIUS, RAYWHITE);
    }

// --- Paddles: ---

    const float THICK_PX = 10.0f;

    for (int a = 0; a < NUM_AGENTS; ++a) {
        const Agent* ag = &env->agents_state[a];
        if (ag->eliminated) continue;

        const Bumper* bp = &ag->bumper;
        //REDO BUMPER DRAWING

        // inward = -outward
        Vector2f n_out = outward_normal_from_rotation(bp->rotation);
        Vector2f n_in  = (Vector2f){ -n_out.x, -n_out.y };

        if (ag->wave_active) {
            Color wave = AGENT_COLORS[a]; wave.a = 160;
            Vector2 c2 = (Vector2){ ag->wave_bumper.position.x, ag->wave_bumper.position.y };
        }
    }
    

    /* TODO: Create a toggleable vector line that is displayed when a ball collides with a 
    paddle highlighting the direction and intensity of the reflection.
    */

}


void c_render(pufferball* env, Client* c) {
    // Build colored lives readout
    // T (red), R (cyan), B (pink), L (yellow)
    const int fontSize = 24;
    int x = c->hud_top_px.x + 12;
    int y = c->hud_top_px.y + 10;
    BeginDrawing();
    ClearBackground(BLACK);
    DrawRectangle(c->hud_top_px.x, c->hud_top_px.y,
                  c->hud_top_px.w, c->hud_top_px.h, (Color){20,20,28,255});

    char buf[32];

    // Top
    snprintf(buf, sizeof(buf), "T:%d  ", env->agents_state[0].lives);
    DrawText(buf, x, y, fontSize, (Color){255, 64, 64, 255});
    x += MeasureText(buf, fontSize);

    // Right
    snprintf(buf, sizeof(buf), "R:%d  ", env->agents_state[1].lives);
    DrawText(buf, x, y, fontSize, (Color){0, 255, 255, 255});
    x += MeasureText(buf, fontSize);

    // Bottom
    snprintf(buf, sizeof(buf), "B:%d  ", env->agents_state[2].lives);
    DrawText(buf, x, y, fontSize, PINK);
    x += MeasureText(buf, fontSize);

    // Left
    snprintf(buf, sizeof(buf), "L:%d", env->agents_state[3].lives);
    DrawText(buf, x, y, fontSize, YELLOW);

    // World view
    BeginScissorMode(c->arena_px.x, c->arena_px.y,
                     c->arena_px.w, c->arena_px.h);
    BeginMode2D(c->cam);
    draw_world_2d(env);                   // draw in world units
    DrawRectangleLines(0, 0, c->world_size, c->world_size, GRAY);
    EndMode2D();
    EndScissorMode();
    EndDrawing();
}


void c_close(pufferball* env, Client* c) {
    (void)env; // currently unused
    if (c) {
        client_destroy(c);
        c = NULL;
    }
    // If you later malloc env->observations/actions/rewards/terminals, free them here.
}


void compute_observations(pufferball* env);


//Balls

void spawn_ball(pufferball* env) {
    
    int idx = alloc_ball_slot(env);
    if (idx < 0) return;

    int corner = rand() % 4;
    Vector2f corner_spawn;
    switch (corner) {
        case 0: corner_spawn = (Vector2f) { (float)env->arena_x, (float)env->arena_y}; break; //TL
        case 1: corner_spawn = (Vector2f) { (float)env->arena_right, (float)env->arena_y}; break; //TR
        case 2: corner_spawn = (Vector2f) { (float)env->arena_right, (float)env->arena_bottom}; break; //BR
        case 3: corner_spawn = (Vector2f) { (float)env->arena_x, (float)env->arena_bottom}; break; //BL
    }

    Vector2f arena_center = env->arena_center;
    Vector2f dir = v2_norm((Vector2f){arena_center.x - corner_spawn.x, arena_center.y - corner_spawn.y});
    float jitter = rnd_range(-SPAWN_ANGLE_JITTER, SPAWN_ANGLE_JITTER);
    dir = v2_rot(dir, jitter);
    Vector2f pos = (Vector2f){ corner_spawn.x + dir.x*kSpawnOffset, corner_spawn.y + dir.y*kSpawnOffset };
    float speed = rnd_range(BALL_SPEED_MIN, BALL_SPEED_MAX);
    Vector2f vel = (Vector2f){ dir.x * speed, dir.y * speed };

    Ball* b = &env->balls[idx];
    b->position = pos;
    b->velocity = vel;
    b->active = true;
}

void update_ball_positions(pufferball* env, float dt){
    
    //Loop through all active balls
    for (int i = 0; i < MAX_BALLS; ++i) {
        Ball* b = &env->balls[i];
        if (!b->active) continue;

        //Integrate Position
        b->position.x += b->velocity.x * dt;
        b->position.y += b->velocity.y * dt;

        // Optionally: Check if ball left arena entirely (out of bounds fail-safe)
        if (b->position.x < env->arena_x - 100 ||
            b->position.x > env->arena_right + 100 ||
            b->position.y < env->arena_y - 100 ||
            b->position.y > env->arena_bottom + 100) {
                b->active = false;
            }
    }
}

void handle_ball_collisions(pufferball* env) {
    // Ball vs. Wall (+ corners first)
    for (int i = 0; i < MAX_BALLS; ++i) {
        Ball* b = &env->balls[i];
        if (!b->active) continue;

        // 1) Corner reflective circles FIRST
        if (handle_corner_reflectors(env, b)) {
            continue;
        }

        // Precompute mouth spans
        const float gx_min = goal_min_x(env), gx_max = goal_max_x(env);
        const float gy_min = goal_min_y(env), gy_max = goal_max_y(env);

        // 2) Then walls, but only outside the goal mouths
        // Left wall
        if (b->position.x - BALL_RADIUS < env->arena_x) {
            float y = b->position.y;
            bool in_mouth = (y >= gy_min && y <= gy_max);
            if (!in_mouth) {
                b->position.x = env->arena_x + BALL_RADIUS;
                b->velocity.x *= -1.0f;
            }
        }
        // Right wall
        else if (b->position.x + BALL_RADIUS > env->arena_right) {
            float y = b->position.y;
            bool in_mouth = (y >= gy_min && y <= gy_max);
            if (!in_mouth) {
                b->position.x = env->arena_right - BALL_RADIUS;
                b->velocity.x *= -1.0f;
            }
        }

        // Top wall
        if (b->position.y - BALL_RADIUS < env->arena_y) {
            float x = b->position.x;
            bool in_mouth = (x >= gx_min && x <= gx_max);
            if (!in_mouth) {
                b->position.y = env->arena_y + BALL_RADIUS;
                b->velocity.y *= -1.0f;
            }
        }
        // Bottom wall
        else if (b->position.y + BALL_RADIUS > env->arena_bottom) {
            float x = b->position.x;
            bool in_mouth = (x >= gx_min && x <= gx_max);
            if (!in_mouth) {
                b->position.y = env->arena_bottom - BALL_RADIUS;
                b->velocity.y *= -1.0f;
            }
        }
    }

    // Bumpers (static + wave)
    for (int i = 0; i < MAX_BALLS; ++i) {
        Ball* b = &env->balls[i];
        if (!b->active) continue;

        for (int a = 0; a < 4; ++a) {
            Agent* ag = &env->agents_state[a];
            if (!ag->eliminated) {
                if (check_ball_bumper_collision(b, &ag->bumper)) {
                    reflect_ball(b, &ag->bumper);
                }
            }
            if (ag->wave_active) {
                if (check_ball_bumper_collision(b, &ag->wave_bumper)) {
                    reflect_ball(b, &ag->wave_bumper);
                    b->velocity.x *= WAVE_SPEED_BOOST;
                    b->velocity.y *= WAVE_SPEED_BOOST;
                }
            }
        }
    }

    // Ball vs Ball
    for (int i = 0; i < MAX_BALLS; ++i) {
        Ball* a = &env->balls[i];
        if (!a->active) continue;

        for (int j = i + 1; j < MAX_BALLS; ++j) {
            Ball* b = &env->balls[j];
            if (!b->active) continue;

            float dx = b->position.x - a->position.x;
            float dy = b->position.y - a->position.y;
            float dist_sq = dx * dx + dy * dy;
            float radius_sum = BALL_RADIUS * 2.0f;

            if (dist_sq < radius_sum * radius_sum) {
                float dist = sqrtf(dist_sq);
                if (dist == 0) dist = 0.01f;
                float nx = dx / dist;
                float ny = dy / dist;

                float vx = b->velocity.x - a->velocity.x;
                float vy = b->velocity.y - a->velocity.y;
                float rel_vel = vx * nx + vy * ny;
                if (rel_vel > 0) continue;

                float impulse = -rel_vel;

                a->velocity.x -= impulse * nx;
                a->velocity.y -= impulse * ny;
                b->velocity.x += impulse * nx;
                b->velocity.y += impulse * ny;

                float overlap = radius_sum - dist;
                float correction = overlap / 2.0f;
                a->position.x -= nx * correction;
                a->position.y -= ny * correction;
                b->position.x += nx * correction;
                b->position.y += ny * correction;
            }
        }
    }
}


//Bumpers
bool check_ball_bumper_collision(Ball* ball, Bumper* bumper){
    float dx = ball->position.x - bumper->position.x;
    float dy = ball->position.y - bumper->position.y;

    // Rotate into bumper-local frame (R^T)
    float c = cosf(bumper->rotation);
    float s = sinf(bumper->rotation);
    float lx = c*dx + s*dy;
    float ly = -s*dx + c*dy;


}


void reflect_ball(Ball* ball, const Bumper* bumper) {
    float dx = ball->position.x - bumper->position.x;
    float dy = ball->position.y - bumper->position.y;
    float c = cosf(bumper->rotation);
    float s = sinf(bumper->rotation);
    float lx = c*dx + s*dy;
    float ly = -s*dx + c*dy;
    }



//Agents
void update_agent_states(pufferball* env, float dt){
    for (int i = 0; i < 4; ++i) {
        Agent* a = &env->agents_state[i];

        //Tick amplify cooldown
        if (a->wave_cooldown > 0.0f){
            a->wave_cooldown -= dt;
            if (a->wave_cooldown < 0.0f) a->wave_cooldown = 0.0f;
        }

        //Keep the static bumper's velocity in sync with agent motion (if any)
        a->bumper.velocity = a->velocity;

        //Wave propegation
        if (a->wave_active){
            a->wave_time += dt;

            Vector2f n_out = outward_normal_from_rotation(a->bumper.rotation);
            a->wave_bumper.position.x += n_out.x * WAVE_SPEED_PX_S * dt;
            a->wave_bumper.position.y += n_out.y * WAVE_SPEED_PX_S * dt;

            //Optional: Keep wave_bumper's velocity for influence in reflections
            a->wave_bumper.velocity.x = n_out.x * WAVE_SPEED_PX_S;
            a->wave_bumper.velocity.y = n_out.y * WAVE_SPEED_PX_S;

            //Expire wave after duration
            if (a->wave_time >= WAVE_DURATION_SEC) {
                a->wave_active = false;
            }

        }
    }
}

bool activate_wave(Agent* agent) {
    if (agent->eliminated){ 
        return false;
    }
    if (agent->wave_cooldown > 0.0f) {
        return false;
    }

    agent->wave_active = true;
    agent->wave_time = 0.0f;
    agent-> wave_cooldown = AMP_COOLDOWN_SEC;
    agent->wave_bumper = agent->bumper;
    agent->wave_bumper.length *= WAVE_RANGE_SCALE;
    agent->wave_bumper.state_timer = WAVE_DURATION_SEC;
    agent->wave_bumper.position = agent->bumper.position;

    return true;
}

void check_goal_hit(pufferball* env){
    const float gx_min = goal_min_x(env);
    const float gx_max = goal_max_x(env);
    const float gy_min = goal_min_y(env);
    const float gy_max = goal_max_y(env);

    const GoalSpec G[4] = {
        // agent, axis, dir, boundary,           other_min, other_max
        { 0,     1,    -1,  (float)env->arena_y,     gx_min,   gx_max }, // TOP: y min, moving up (v_y < 0)
        { 1,     0,    +1,  (float)env->arena_right, gy_min,   gy_max }, // RIGHT: x max, moving right (v_x > 0)
        { 2,     1,    +1,  (float)env->arena_bottom,gx_min,   gx_max }, // BOTTOM: y max, moving down (v_y > 0)
        { 3,     0,    -1,  (float)env->arena_x,     gy_min,   gy_max }, // LEFT: x min, moving left (v_x < 0)
    };

    for (int i = 0; i < MAX_BALLS; ++i) {
        Ball* b = &env->balls[i];
        if(!b->active) continue;

        //test against each goal
        for (int g = 0; g < 4; ++g) {
            const GoalSpec* s = &G[g];
            Agent* a = &env->agents_state[s->agent];
            if (a->eliminated) continue;

            const float p_axis = v2_axis(b->position, s->axis);
            const float v_axis = v2_axis(b->velocity, s->axis);
            const float p_orth = v2_axis_orth(b->position, s->axis);

            //Checks if the ball is moving towards the active goal
            if (s->dir > 0 ? (v_axis <= 0.0f) : (v_axis >= 0.0f)) continue;

            //Checks if the ball's edge has crossed the goal
            const float edge = p_axis + s->dir * BALL_RADIUS;
            if (s->dir > 0 ? (edge < s->boundary) : (edge > s->boundary)) continue;

            //Checks to make sure the ball didn't hit the corner spawns
            if(!betweenf(p_orth, s->other_min, s->other_max)) continue;

            a->lives -= 1;
            if (a->lives <= 0) eliminate_agent(env, s->agent);
            b->active = false;
            break;
        }
    }
}

void eliminate_agent(pufferball* env, int agent_id){
    
    if (agent_id < 0 || agent_id >= 4) return;
    Agent* a = &env->agents_state[agent_id];
    if(a->eliminated) return;
    a->eliminated = true;
    a->wave_active = false;
    a->wave_time = 0.0f;
    a->bumper.state_timer = 0.0f;
    a->velocity.x = a->velocity.y = 0.0f;
    check_game_over(env);
}

void check_game_over(pufferball* env){
    
    int alive_id = -1;
    int alive_count = 0;
    
    for (int i = 0; i < 4; ++i) {
        if (!env->agents_state[i].eliminated && env->agents_state[i].lives > 0) {
            alive_id = i;
            alive_count++;
        }
    }

    if (alive_count <= 1){
        env->game_over = true;
        env->winner_id = (alive_count == 1) ? alive_id : -1;
    }
}

#endif // PUFFERBALL_H
