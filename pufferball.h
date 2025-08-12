#ifndef PUFFERBALL_H
#define PUFFERBALL_H

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "raylib.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>   




//Ball Constants
#define MAX_BALLS           10
#define BALL_SPEED_MIN      220.0f
#define BALL_SPEED_MAX      360.0f
#define BALL_RADIUS         5

//Ball Spawn Consts
#define SPAWN_ANGLE_JITTER  (float)(M_PI / 6) // +-30 degrees in radians
#define CORNER_RADIUS       40.0f
#define SPAWN_MARGIN        1.0f

#define AMP_FACTOR          1.25f //Speed multiplier when amplified
#define BUMPER_INFLUENCE    0.50f //How much bump velocity biases reflection
#define SEPARATION_EPSILON  0.75f //Sticking Avoidance

#define AMP_COOLDOWN_SEC    4.0f
#define WAVE_DURATION_SEC   0.40f
#define WAVE_SPEED_PX_S     900.0f
#define WAVE_RANGE_SCALE    1.35f
#define WAVE_SPEED_BOOST    1.35f


#ifndef M_PI
#define M_PI                3.14159265
#endif


//Bumper States
enum { BUMPER_NORMAL = 0, BUMPER_AMPLIFIED = 1 };




typedef struct {
    float perf;
    float score;
    float episode_return;
    float episode_length;
    float n;
} Log;

enum { ARENA_PX = 800 };
enum { HUD_TOP_PX = 75 };


typedef struct { int x, y ,w, h; } RectI;

typedef struct {
    int win_w, win_h, world_size;
    RectI hud_top_px, arena_px;
    float world_to_px;
    Camera2D cam;
} Client;


// 2D vector
typedef struct { float x, y; } Vector2f;

// Construct & basic ops
static inline Vector2f v2(float x, float y)          { return (Vector2f){x,y}; }
static inline Vector2f v2_add(Vector2f a, Vector2f b){ return v2(a.x+b.x, a.y+b.y); }
static inline Vector2f v2_sub(Vector2f a, Vector2f b){ return v2(a.x-b.x, a.y-b.y); }
static inline Vector2f v2_mul(Vector2f v, float s)   { return v2(v.x*s, v.y*s); }

// Length & normalization
static inline float    v2_len_sq(Vector2f v)         { return v.x*v.x + v.y*v.y; }
static inline float    v2_len(Vector2f v)            { return sqrtf(v2_len_sq(v)); }
static inline Vector2f v2_norm(Vector2f v)           { float L=v2_len(v); return (L>0)? v2(v.x/L, v.y/L) : v2(0,0); }

// Dot & rotation
static inline float    v2_dot(Vector2f a, Vector2f b){ return a.x*b.x + a.y*b.y; }
static inline Vector2f v2_rot(Vector2f v, float ang) { float c=cosf(ang), s=sinf(ang); return v2(c*v.x - s*v.y, s*v.x + c*v.y); }

// Axis helpers (for generic X/Y logic like goals)
static inline float    v2_axis(Vector2f v, int axis)       { return axis ? v.y : v.x; }
static inline float    v2_axis_orth(Vector2f v, int axis)  { return axis ? v.x : v.y; }




typedef struct {
    Vector2f    position;   //ellipse center
    float       rx;         //ellipse radius along local x
    float       ry;         //ellipse radius along local y
    float       rotation;   //radians; local +x, +y 
    
    //Arc Limits in Elliptical Local Space

    float angle_start;  //e.g. -PI/2
    float angle_end;    //e.g. PI/2
    
    
    Vector2f        velocity; //Influence from bumper motion
    unsigned char   state;
    float           state_timer;

} Bumper;

typedef struct {
    int agent;
    int axis;
    int dir;
    float boundary;
    float other_min;
    float other_max;
} GoalSpec;

typedef struct {
    
    Vector2f        position;
    Vector2f        velocity;

    int             lives;
    bool            eliminated;
    
    Bumper          bumper;
    
    float           amplify_cooldown;

    //Wave Function
    bool            wave_active;
    float           wave_time;
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
    

    // Internal Sim State

    int size; //World Side Length

    Agent agents_state[4];
    Ball balls[MAX_BALLS];

    float time_since_last_spawn;
    bool game_over;
    int winner_id;

    //Arena Geometry
    int arena_x;
    int arena_y;
    int arena_right;
    int arena_bottom;
    Vector2f arena_center;

} pufferball;

// ---- Forward declarations ----

// core lifecycle
void init(pufferball* env);
void c_reset(pufferball* env);
void c_step(pufferball* env);
void c_render(pufferball* env);
void c_close(pufferball* env);

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

//UI Layout
static inline Client* client_create(int world_size) {
    SetConfigFlags(FLAG_MSAA_4X_HINT);
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

typedef void (*draw_world_fn)(const void* env);

static inline void client_render(const Client* c, const void* env, draw_world_fn draw_world, const char* hud_text) {
    BeginDrawing();
    ClearBackground(BLACK);

    // HUD
    DrawRectangle(c->hud_top_px.x, c->hud_top_px.y, c->hud_top_px.w, c->hud_top_px.h, (Color){20,20,28,255});
    if (hud_text) DrawText(hud_text, c->hud_top_px.x + 12, c->hud_top_px.y + 10, 24, RAYWHITE);

    // World
    BeginScissorMode(c->arena_px.x, c->arena_px.y, c->arena_px.w, c->arena_px.h);
    BeginMode2D(c->cam);
    if (draw_world) draw_world(env);                   // draw in world units
    DrawRectangleLines(0,0, c->world_size, c->world_size, GRAY);
    EndMode2D();
    EndScissorMode();

    EndDrawing();
}


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

static inline bool angle_in_range(float theta, float start, float end) {
    theta = wrap_pi(theta);
    start = wrap_pi(start);
    end = wrap_pi(end);

    if (start <= end) {
        return (theta >= start && theta <= end);
    } else {
        return (theta >= start || theta <= end);
    }
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


enum { NUM_AGENTS = 4 };
static const int   kStartingLives   = 15;
static const float kFixedDt         = 1.0f/60.0f;  // deterministic sim step
static const float kSpawnCooldown   = 0.75f;       // seconds with no balls before auto-spawn
static const float kBumperRx        = 42.0f;
static const float kBumperRy        = 14.0f;
static const float kBumperArcStart  = -M_PI/3.0f;  // ~120° forward-facing arc
static const float kBumperArcEnd    =  M_PI/3.0f;

// Optional: keep the client private to this file
static Client* s_client = NULL;

// Small utility
static inline void zero_balls(pufferball* env) {
    for (int i = 0; i < MAX_BALLS; ++i) env->balls[i].active = false;
}

// Place 4 agents centered on each side, facing inward
static void init_agents_on_walls(pufferball* env) {
    const int   S  = env->size;
    const float cx = S * 0.5f;

    // Top (agent 0): outward normal points up, so use rotation = PI
    Agent* top = &env->agents_state[0];
    *top = (Agent){0};
    top->position = (Vector2f){ cx, 0.0f };
    top->lives = kStartingLives;
    top->eliminated = false;
    top->bumper.position    = top->position;
    top->bumper.rx          = kBumperRx;
    top->bumper.ry          = kBumperRy;
    top->bumper.rotation    = (float)M_PI;
    top->bumper.angle_start = kBumperArcStart;
    top->bumper.angle_end   = kBumperArcEnd;
    top->bumper.state       = BUMPER_NORMAL;

    // Right (agent 1): outward +X, rotation = -PI/2
    Agent* right = &env->agents_state[1];
    *right = (Agent){0};
    right->position = (Vector2f){ (float)S, cx };
    right->lives = kStartingLives;
    right->eliminated = false;
    right->bumper.position    = right->position;
    right->bumper.rx          = kBumperRx;
    right->bumper.ry          = kBumperRy;
    right->bumper.rotation    = -(float)M_PI/2.0f;
    right->bumper.angle_start = kBumperArcStart;
    right->bumper.angle_end   = kBumperArcEnd;
    right->bumper.state       = BUMPER_NORMAL;

    // Bottom (agent 2): outward +Y, rotation = 0
    Agent* bottom = &env->agents_state[2];
    *bottom = (Agent){0};
    bottom->position = (Vector2f){ cx, (float)S };
    bottom->lives = kStartingLives;
    bottom->eliminated = false;
    bottom->bumper.position    = bottom->position;
    bottom->bumper.rx          = kBumperRx;
    bottom->bumper.ry          = kBumperRy;
    bottom->bumper.rotation    = 0.0f;
    bottom->bumper.angle_start = kBumperArcStart;
    bottom->bumper.angle_end   = kBumperArcEnd;
    bottom->bumper.state       = BUMPER_NORMAL;

    // Left (agent 3): outward -X, rotation = +PI/2
    Agent* left = &env->agents_state[3];
    *left = (Agent){0};
    left->position = (Vector2f){ 0.0f, cx };
    left->lives = kStartingLives;
    left->eliminated = false;
    left->bumper.position    = left->position;
    left->bumper.rx          = kBumperRx;
    left->bumper.ry          = kBumperRy;
    left->bumper.rotation    = (float)M_PI/2.0f;
    left->bumper.angle_start = kBumperArcStart;
    left->bumper.angle_end   = kBumperArcEnd;
    left->bumper.state       = BUMPER_NORMAL;
}

void init(pufferball* env) {
    
    srand((unsigned)time(NULL));
    
    // World size in world units (align it 1:1 with your ARENA_PX for simplicity)
    env->size = ARENA_PX;

    // Arena in world coordinates (do NOT bake HUD offsets here)
    env->arena_x      = 0;
    env->arena_y      = 0;
    env->arena_right  = env->size;
    env->arena_bottom = env->size;
    env->arena_center = (Vector2f){ env->size * 0.5f, env->size * 0.5f };

    env->log = (Log){0};
    env->time_since_last_spawn = 0.0f;
    env->game_over = false;
    env->winner_id = -1;

    zero_balls(env);
    init_agents_on_walls(env);

    // Optionally, prime with a first ball
    spawn_ball(env);
}

void c_reset(pufferball* env) {
    env->log = (Log){0};
    env->game_over = false;
    env->winner_id = -1;
    env->time_since_last_spawn = 0.0f;

    // Keep arena dims the same; just reset agents/balls
    init_agents_on_walls(env);
    zero_balls(env);
    spawn_ball(env);
}

void c_step(pufferball* env) {
    if (env->game_over) return;

    const float dt = kFixedDt;

    // Agents (cooldowns, wave propagation, bumper velocity sync, etc.)
    update_agent_states(env, dt);

    // Ball integration & collisions
    update_ball_positions(env, dt);
    handle_ball_collisions(env);

    // Scoring & elimination
    check_goal_hit(env);
    if (env->game_over) return;

    // Auto-spawn logic: if no active balls, start a short cooldown then spawn
    bool any_active = false;
    for (int i = 0; i < MAX_BALLS; ++i) {
        if (env->balls[i].active) { any_active = true; break; }
    }

    if (!any_active) {
        env->time_since_last_spawn += dt;
        if (env->time_since_last_spawn >= kSpawnCooldown) {
            env->time_since_last_spawn = 0.0f;
            spawn_ball(env);
        }
    } else {
        env->time_since_last_spawn = 0.0f;
    }

    // Observations can wait; call your stub if you want to keep the interface consistent
    // compute_observations(env);
}

// Simple world drawer used by c_render (balls + goal mouths + optional corner circles)
static void draw_world_2d(const void* ptr) {
    const pufferball* env = (const pufferball*)ptr;

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

    // Bumper direction indicators (short normal lines)
    for (int a = 0; a < NUM_AGENTS; ++a) {
        const Agent* ag = &env->agents_state[a];
        if (ag->eliminated) continue;
        Vector2f n = outward_normal_from_rotation(ag->bumper.rotation);
        Vector2f p0 = ag->bumper.position;
        Vector2f p1 = (Vector2f){ p0.x + n.x * 20.0f, p0.y + n.y * 20.0f };
        DrawLine((int)p0.x, (int)p0.y, (int)p1.x, (int)p1.y, (Color){200,80,80,255});
    }
}

void c_render(pufferball* env) {
    if (!s_client) {
        s_client = client_create(env->size);
        if (!s_client) return;
    }

    // Simple HUD line
    char hud[128];
    snprintf(hud, sizeof(hud), "Lives  T:%d  R:%d  B:%d  L:%d",
        env->agents_state[0].lives,
        env->agents_state[1].lives,
        env->agents_state[2].lives,
        env->agents_state[3].lives);

    client_render(s_client, env, draw_world_2d, hud);
}



void c_close(pufferball* env) {
    (void)env; // currently unused
    if (s_client) {
        client_destroy(s_client);
        s_client = NULL;
    }
    // If you later malloc env->observations/actions/rewards/terminals, free them here.
}


void compute_observations(pufferball* env);


//Balls

void spawn_ball(pufferball* env) {
    
    const float spawn_offset = CORNER_RADIUS + SPAWN_MARGIN;
    
    int idx = alloc_ball_slot(env);
    if (idx < 0) return;

    int corner = rand() % 4;
    Vector2f center;
    switch (corner) {
        case 0: center = (Vector2f) { (float)env->arena_x, (float)env->arena_y}; break; //TL
        case 1: center = (Vector2f) { (float)env->arena_right, (float)env->arena_y}; break; //TR
        case 2: center = (Vector2f) { (float)env->arena_right, (float)env->arena_bottom}; break; //BR
        case 3: center = (Vector2f) { (float)env->arena_x, (float)env->arena_bottom}; break; //BL
    }

    //Aim directly toward arena center then add small random angle jitter
    Vector2f arena_center = env->arena_center;
    Vector2f dir = v2_norm((Vector2f){arena_center.x - center.x, arena_center.y - center.y});
    float jitter = rnd_range(-SPAWN_ANGLE_JITTER, SPAWN_ANGLE_JITTER);
    dir = v2_rot(dir, jitter);

    //Spawn just inside the reflective corner circle boundary along dir
    Vector2f pos = (Vector2f){ center.x + dir.x*spawn_offset, center.y + dir.y*spawn_offset };

    //Speed
    float speed = rnd_range(BALL_SPEED_MIN, BALL_SPEED_MAX);
    Vector2f vel = (Vector2f){ dir.x * speed, dir.y * speed };

    //Commit
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
            // (returns immediately after a corner bounce to avoid double-bounce)
            if (handle_corner_reflectors(env, b)) {
                // optional: clamp_speed(b);
                continue;
            }
    
            // 2) Then walls
            // Left / Right
            if (b->position.x - BALL_RADIUS < env->arena_x) {
                b->position.x = env->arena_x + BALL_RADIUS;
                b->velocity.x *= -1;
            }
            else if (b->position.x + BALL_RADIUS > env->arena_right) {
                b->position.x = env->arena_right - BALL_RADIUS;
                b->velocity.x *= -1;
            }
    
            // Top / Bottom
            if (b->position.y - BALL_RADIUS < env->arena_y) {
                b->position.y = env->arena_y + BALL_RADIUS;
                b->velocity.y *= -1;
            }
            else if (b->position.y + BALL_RADIUS > env->arena_bottom) {
                b->position.y = env->arena_bottom - BALL_RADIUS;
                b->velocity.y *= -1;
            }
    
            // optional: clamp_speed(b);
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
                        // optional: clamp_speed(b);
                    }
                }
                if (ag->wave_active) {
                    if (check_ball_bumper_collision(b, &ag->wave_bumper)) {
                        reflect_ball(b, &ag->wave_bumper);
                        b->velocity.x *= WAVE_SPEED_BOOST;
                        b->velocity.y *= WAVE_SPEED_BOOST;
                        // optional: clamp_speed(b);
                    }
                }
            }
        }
    // Include a Toggable Uniform Grid (or add this to another project (Touhou style game???))

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
                //Simple Elastic Collision
                
                //Normalize Collision Normal
                float dist = sqrtf(dist_sq);
                if (dist == 0) dist = 0.01f;
                float nx = dx / dist;
                float ny = dy / dist;

                //Relative Velocity
                float vx = b->velocity.x - a->velocity.x;
                float vy = b->velocity.y - a->velocity.y;

                //Dot Product
                float rel_vel = vx * nx + vy * ny;

                //Ignore if separating
                if (rel_vel > 0) continue;

                //Simple impulse scalar
                float impulse = -rel_vel;
                // General form if mass is introduced later
                // float impulse = -(1 + e) * rel_vel / (1/m1 + 1/m2); 

                //Apply Impulse
                a->velocity.x -= impulse * nx;
                a->velocity.y -= impulse * ny;
                b->velocity.x += impulse * nx;
                b->velocity.y += impulse * ny;
                
                //Positional Correction -- Sticking / Exploding Fix
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

    // Rotate by -rotation to enter ellipse-local frame
    float c = cosf(bumper->rotation);
    float s = sinf(bumper->rotation);
    float lx = c * dx + s * dy;
    float ly = -s * dx + c * dy;

    // Scale to unit circle space
    float qx = lx / bumper->rx;
    float qy = ly / bumper->ry;

    // Polar in unit-circle space
    float r = sqrtf(qx * qx + qy * qy);
    float theta = atan2f(qy, qx);

    // Check arc span
    if (!angle_in_range(theta, bumper->angle_start, bumper->angle_end)){
        return false;
    }

    //Inflating the ellipse by ball radius, this is so that the ellipse hits the edge of the ball not the center, but it still seems fishy
    float r_inflate = fmaxf(BALL_RADIUS / bumper->rx, BALL_RADIUS / bumper->ry);
    
    return fabsf(r - 1.0f) <= r_inflate;
}

void reflect_ball(Ball* ball, const Bumper* bumper) {
    //Step 1: Vector from bumper center to ball (world space)
    float dx = ball->position.x - bumper->position.x;
    float dy = ball ->position.y - bumper->position.y;

    //Step 2: rotate into ellipse local frame (apply R^T)
    float c = cosf(bumper->rotation);
    float s = sinf(bumper->rotation);
    float lx = c*dx + s*dy;
    float ly = -s*dx + c*dy;

    //Step 3: scale to unit circle space
    float qx = lx / bumper->rx;
    float qy = ly / bumper->ry;

    //Guard: If the ball center is exactly at ellipse center in local space
    float qlen = sqrtf(qx*qx + qy*qy); 
    if (qlen == 0.0f) {
        Vector2f n_world = (Vector2f){ -s, c };
        n_world = v2_norm(n_world);

        //Relative Velocity
        Vector2f v_rel = v2_sub(ball->velocity, v2_mul(bumper->velocity, BUMPER_INFLUENCE));
        float vn = v2_dot(v_rel, n_world);
        if (vn < 0.0f) {
            //reflect
            Vector2f v_ref = v2_sub (v_rel, v2_mul(n_world, 2.0f*vn));
            Vector2f v_new = v2_add(v_ref, v2_mul(bumper->velocity, BUMPER_INFLUENCE));

            //apply state scaling
            float scale = 1.0f;
            if (bumper->state == BUMPER_AMPLIFIED){
                scale = AMP_FACTOR;
            }

            ball->velocity.x = v_new.x * scale;
            ball->velocity.y = v_new.y * scale;

            //small separation to avoid repenetration
            ball->position.x += n_world.x * SEPARATION_EPSILON;
            ball->position.y += n_world.y * SEPARATION_EPSILON;
        }
        return;
    }

    //Step 4: nearest boundary point on unit circle is radial projection
    float ux = qx / qlen;
    float uy = qy / qlen;

    //Map boundary poin tback to ellipse local coords
    float ex = bumper->rx * ux;
    float ey = bumper->ry * uy;

    //Step 5: local space normal = gradient of F(x,y) = x^2 / rx^2 + y^2 / ry^2 -1
    // ∇F ∝ (2x/rx^2, 2y/ry^2) -> normal direction:
    float nxL = ex / (bumper->rx * bumper->rx);
    float nyL = ey / (bumper->ry * bumper->ry);
    //Normalize local normal
    float nL_len = sqrtf(nxL*nxL + nyL*nyL);
    if (nL_len > 0.0f) { nxL /= nL_len; nyL /= nL_len; }

    //Step 6: rotate normal back to world (apply R)
    Vector2f n_world = (Vector2f){
        c*nxL - s*nyL,
        s*nxL + c*nyL
    };
    n_world = v2_norm(n_world);

    // Step 7: reflect relative velocity about world normal
    Vector2f v_rel = v2_sub(ball->velocity, v2_mul(bumper->velocity, BUMPER_INFLUENCE));
    float vn = v2_dot(v_rel, n_world);

    // Only reflect if approaching the surface
    if (vn >= 0.0f) return;

    Vector2f v_ref = v2_sub(v_rel, v2_mul(n_world, 2.0f*vn));
    Vector2f v_new = v2_add(v_ref, v2_mul(bumper->velocity, BUMPER_INFLUENCE));

    // Step 8: apply bumper state scaling
    float scale = 1.0f;
    if (bumper->state == BUMPER_AMPLIFIED){ 
        scale = AMP_FACTOR;
    }

    ball->velocity.x = v_new.x * scale;
    ball->velocity.y = v_new.y * scale;

    // Step 9: small positional correction outward along normal
    // (Optional: you could compute exact penetration, but a small epsilon works well.)
    ball->position.x += n_world.x * SEPARATION_EPSILON;
    ball->position.y += n_world.y * SEPARATION_EPSILON;

}

//Agents
void update_agent_states(pufferball* env, float dt){
    for (int i = 0; i < 4; ++i) {
        Agent* a = &env->agents_state[i];

        //Tick amplify cooldown
        if (a->amplify_cooldown > 0.0f){
            a->amplify_cooldown -= dt;
            if (a->amplify_cooldown < 0.0f) a->amplify_cooldown = 0.0f;
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

bool activate_amplify(Agent* agent) {
    if (agent->eliminated){ 
        return false;
    }
    if (agent->amplify_cooldown > 0.0f) {
        return false;
    }

    //Initialize wave state
    agent->wave_active = true;
    agent->wave_time = 0.0f;
    agent-> amplify_cooldown = AMP_COOLDOWN_SEC;

    agent->wave_bumper = agent->bumper;
    agent->wave_bumper.rx *= WAVE_RANGE_SCALE;
    agent->wave_bumper.ry *= WAVE_RANGE_SCALE;
    agent->wave_bumper.state = BUMPER_AMPLIFIED;
    agent->wave_bumper.state_timer = WAVE_DURATION_SEC;

    agent->wave_bumper.position = agent->bumper.position;

    return true;
}


//Game Rules
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

            //must be moving toward the tail
            if (s->dir > 0 ? (v_axis <= 0.0f) : (v_axis >= 0.0f)) continue;

            //cross the boundary with ball radius considered
            const float edge = p_axis + s->dir * BALL_RADIUS;
            if (s->dir > 0 ? (edge < s->boundary) : (edge > s->boundary)) continue;

            //within the goal mouth span on the orthogonal axis
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

    //Cancel Wave
    a->wave_active = false;
    a->wave_time = 0.0f;

    //Reset Bumper State
    a->bumper.state = BUMPER_NORMAL;
    a->bumper.state_timer = 0.0f;

    //Optional: Zero Cooldowns / Velocity

    a->amplify_cooldown = 0.0f;
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
