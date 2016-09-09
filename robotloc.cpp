#include <cstdint>
#include <cstdio>
#include <cassert>

#include <fstream>

using std::uint32_t;
using std::uint64_t;

constexpr int WIDTH = 20, HEIGHT = 8;

const char map[WIDTH * HEIGHT + 1] = \
/*
    "####################"
    "#          #       #"
    "# #### ### # ##### #"
    "# #    # #   #     #"
    "#   ##   ##### #####"
    "###    ###   #     #"
    "#                  #"
    "####################";
*/
    "####################"
    "#                  #"
    "# #### ###   ##### #"
    "#        #         #"
    "#              #####"
    "###    ###   #     #"
    "#                  #"
    "####################";

// stored as 32-bit fixed points
const uint32_t DIR_NOISE_CHANCE    = 0x10000000;
const uint32_t DIR_BACK_CHANCE     = 0x01000000;
const uint32_t SENSOR_NOISE_CHANCE = 0x20000000;

// robot location via hidden markov models

struct point {
    int p[2];
};

inline bool operator==(const point &a, const point &b) {
    return a.p[0] == b.p[0] && a.p[1] == b.p[1];
}

enum direction {
    EAST,
    NORTH,
    WEST,
    SOUTH,
    NUM_DIRECTIONS, // also invalid direction
};

const char *const dbg_dir_strings[4] = {"EAST", "NORTH", "WEST", "SOUTH"};

struct observation {
    bool sensor[NUM_DIRECTIONS]; // true if no wall
    direction direction;
};

point move_point(const point &point, direction direction) {
    switch (direction) {
    case EAST:
        return {{point.p[0] + 1, point.p[1]}};
    case NORTH:
        return {{point.p[0], point.p[1] - 1}};
    case WEST:
        return {{point.p[0] - 1, point.p[1]}};
    case SOUTH:
        return {{point.p[0], point.p[1] + 1}};
    case NUM_DIRECTIONS:
        return point;
    }
    return point;
}

typedef unsigned short sample_map[WIDTH * HEIGHT];

inline size_t point_index(const point &point) {
    return point.p[0] + point.p[1] * WIDTH;
}

inline point from_index(size_t index) {
    return {{(int)(index % WIDTH), (int)(index / WIDTH)}};
}

bool is_wall(const point &point, const char *map) {
    return map[point_index(point)] == '#';
}

bool is_invalid(const point &point) {
    return point.p[0] < 0 || point.p[0] >= WIDTH
        || point.p[1] < 0 || point.p[1] >= HEIGHT;
}

// should have all 32 bits entropy
template<typename PRNG> uint32_t next_rand(PRNG *rng) {
    static_assert(sizeof(PRNG)==-1, "next_rand() not implemented for given PRNG type");
    return 4; // chosen by fair dice roll
}

struct bb_rand_ctx {
    uint32_t a, b, c, d;
};

template<> uint32_t next_rand<bb_rand_ctx>(bb_rand_ctx *x) {
    uint32_t \
       e = x->a - ((x->b << 27) | (x->b >> 5));
    x->a = x->b ^ ((x->c << 17) | (x->c >> 15));
    x->b = x->c + x->d;
    x->c = x->d + e;
    x->d = e + x->a;
    return x->d;
}

void bb_rand_init(bb_rand_ctx *x, uint32_t seed) {
    x->a = 0xf1ea5eed;
    x->b = x->c = x->d = seed;
    for (size_t i = 0; i < 20; i++) next_rand(x);
}

bool is_dir_free(const point &point, const char *const map, direction direction) {
    auto move = move_point(point, direction);
    return !is_invalid(move) && !is_wall(move, map);
}

template<typename PRNG> direction move_randomly(const point &point, const char *const map, PRNG *rng) {
    direction possibilities[NUM_DIRECTIONS];
    size_t num_possibilities = 0;
    for (size_t dir = 0; dir < NUM_DIRECTIONS; dir++) {
        if (is_dir_free(point, map, (direction)dir)) possibilities[num_possibilities++] = (direction)dir;
    }
    if (num_possibilities == 0) return NUM_DIRECTIONS;
    if (num_possibilities == 1) return possibilities[0];
    // bit stuff for speed
    static_assert(0x3 == NUM_DIRECTIONS - 1, "NUM_DIRECTIONS must equal 4");
    uint32_t rdir = next_rand(rng) & 0x3;
    if (num_possibilities == 2) return possibilities[rdir & 0x1];
    if (num_possibilities == 4) return possibilities[rdir];
    while (rdir >= num_possibilities) {
        rdir = (direction)(next_rand(rng) & 0x3);
    }
    return possibilities[rdir];
}

observation compute_observation(const point &point, const char *const map, direction obs_dir) {
    observation ret;
    for (int dir = 0; dir < NUM_DIRECTIONS; dir++) {
        ret.sensor[dir] = is_dir_free(point, map, (direction)dir);
    }
    ret.direction = obs_dir;
    return ret;
}

template<typename PRNG> void perturb_observation(observation &observation, PRNG *rng) {
    uint32_t rand = next_rand(rng);
    static_assert(NUM_DIRECTIONS == 4, "NUM_DIRECTIONS must equal 4"); // we use bit stuff
    if (rand < DIR_NOISE_CHANCE) {
        if (rand < DIR_BACK_CHANCE) {
            std::printf("perturbing direction backwards\n");
            observation.direction = (direction)((size_t)observation.direction ^ 2);
        } else {
            std::printf("perturbing direction sideways\n");
            observation.direction = (direction)((size_t)observation.direction ^ 1);
            if (rand & 1) observation.direction = (direction)((size_t)observation.direction ^ 2);;
        }
    }
    for (int dir = 0; dir < NUM_DIRECTIONS; dir++) {
        uint32_t rand = next_rand(rng);
        if (rand < SENSOR_NOISE_CHANCE) {
            std::printf("perturbing sensor %s\n", dbg_dir_strings[dir]);
            observation.sensor[dir] = !observation.sensor[dir];
        }
    }
}

struct locator {
    uint32_t probability[WIDTH * HEIGHT];
};

const point &next_point(point &pt) {
    pt.p[0]++;
    if (pt.p[0] >= WIDTH) {
        pt.p[0] = 0;
        pt.p[1]++;
    }
    return pt;
}

// make sure to renormalize!
uint32_t observation_probability(const observation &from, const observation &to) {
    uint64_t ret = (uint64_t)1 << 32;
    for (int dir = 0; dir < NUM_DIRECTIONS; dir++) {
        if (from.sensor[dir] != to.sensor[dir]) {
            ret = (ret * SENSOR_NOISE_CHANCE) >> 32;
        } else {
            ret = (ret * (((uint64_t)1 << 32) - SENSOR_NOISE_CHANCE)) >> 32;
        }
    }
    static_assert(NUM_DIRECTIONS == 4, "NUM_DIRECTIONS must equal 4");
    switch (from.direction ^ to.direction) {
    case 1:
    case 3:
        ret = (ret * (DIR_NOISE_CHANCE - DIR_BACK_CHANCE)) / 2 >> 32;
        break;
    case 2:
        ret = (ret * DIR_BACK_CHANCE) >> 32;
        break;
    case 0:
        ret = (ret * (((uint64_t)1 << 32)) - DIR_NOISE_CHANCE) >> 32;
        break;
    }
    return (uint32_t)ret;
}

void normalize_probabilities(uint32_t *arr, size_t size) {
    uint64_t sum_prob = 0;
    for (size_t i = 0; i < size; i++) sum_prob += arr[i];
    std::printf("sum prob: %llu\n", sum_prob);
    std::fflush(stdout);
    for (size_t i = 0; i < size; i++) {
        if (arr[i] == sum_prob) arr[i] = (uint32_t)(((uint64_t)1 << 32) - 1);
        else arr[i] = (uint32_t)(((uint64_t)arr[i] << 32) / sum_prob);
    }
    sum_prob = 0;
    for (size_t i = 0; i < size; i++) sum_prob += arr[i];
    std::printf("sum prob: %llu\n", sum_prob);
    std::fflush(stdout);
}

locator update_locator(const locator &src_locator, const char *map, const observation &observation) {
    locator ret = {{0}};
    uint32_t dir_probs[NUM_DIRECTIONS];
    for (point pt = {{0}}; pt.p[1] < HEIGHT; next_point(pt)) {
        if (is_wall(pt, map)) continue;
        size_t num_prob = 0; // number of possible directions to move (calculate probability of transition)
        for (int dir = 0; dir < NUM_DIRECTIONS; dir++) {
            point np = move_point(pt, (direction) dir);
            if (is_invalid(np) || is_wall(np, map)) {
                dir_probs[dir] = 0;
            } else {
                dir_probs[dir] = observation_probability(compute_observation(np, map, (direction) dir), observation);
                num_prob++;
            }
        }
        if (num_prob == 0) continue;
        for (int dir = 0; dir < NUM_DIRECTIONS; dir++) {
            point np = move_point(pt, (direction)dir);
            uint64_t moveprob = ((((uint64_t)dir_probs[dir] * src_locator.probability[point_index(pt)]) / num_prob) >> 32);
            uint64_t newprob = ret.probability[point_index(np)] + moveprob;
            if (newprob >= ((uint64_t)1 << 32)) newprob = ((uint64_t)1 << 32) - 1;
            ret.probability[point_index(np)] = (uint32_t)newprob;
        }
    }
    normalize_probabilities(ret.probability, WIDTH * HEIGHT);
    return ret;
}

void dbg_print_observation(const observation &obs) {
    printf(" EAST: %s\n", obs.sensor[ EAST] ? "yes" : "no");
    printf("NORTH: %s\n", obs.sensor[NORTH] ? "yes" : "no");
    printf(" WEST: %s\n", obs.sensor[ WEST] ? "yes" : "no");
    printf("SOUTH: %s\n", obs.sensor[SOUTH] ? "yes" : "no");
    printf("dir: %s\n", dbg_dir_strings[(size_t)obs.direction]);
}

void write_observation(std::ofstream &out_json, const char *field_name, const observation &obs) {
    out_json << "\"" << field_name << "\":{"
        "\"sensor\":[";
    for (size_t i = 0; i < NUM_DIRECTIONS; i++) {
        if (i) out_json << ",";
        out_json << obs.sensor[i];
    }
    out_json << "],\"direction\":" << (size_t) obs.direction;
    out_json << "},";
}

int main(int argc, char **argv) {
    locator loc = {{0}};
    size_t nspaces = 0;
    for (size_t a = 0; a < WIDTH * HEIGHT; a++) if (!is_wall(from_index(a), map)) nspaces++;
    uint32_t prob = ((uint64_t)1 << 32) / nspaces;
    for (size_t a = 0; a < WIDTH * HEIGHT; a++) loc.probability[a] = prob;
    point pt = {{0, 0}};
    if (is_wall(pt, map)) pt.p[0] = pt.p[1] = 1;
    std::printf("probability: %.12f\n", loc.probability[0] / (double)((uint64_t)1 << 32));
    std::fflush(stdout);
    // direction movements[] = {EAST, EAST, EAST, EAST, EAST, SOUTH, SOUTH, WEST, WEST, WEST, SOUTH, WEST, WEST, NORTH};
    size_t num_movements = 100;
    // rng
    bb_rand_ctx prng;
    bb_rand_init(&prng, 0xDEADBEEF);
    // log movements and probabilities
    std::ofstream out_json("robot.json");
    out_json << "{\"width\":" << WIDTH << ",\"height\":" << HEIGHT << ",\"map\":[";
    for (size_t q = 0; q < WIDTH * HEIGHT; q++) {
        if (q) out_json << ",";
        out_json << is_wall(from_index(q), map);
    }
    out_json << "],\"data\":[";
    for (size_t index = 0; index < num_movements; index++) {
        if (index) out_json << ",";
        direction move_dir = move_randomly(pt, map, &prng);
        out_json << "{";
        std::printf("||> MOVEMENT %llu: %s\n", index + 1, dbg_dir_strings[move_dir]);
        std::fflush(stdout);
        // move around
        pt = move_point(pt, move_dir);
        out_json << "\"location\":[" << pt.p[0] << "," << pt.p[1] << "],";
        assert(!is_invalid(pt) && !is_wall(pt, map));
        observation obs = compute_observation(pt, map, move_dir);
        write_observation(out_json, "obs_real", obs);
        perturb_observation(obs, &prng);
        write_observation(out_json, "obs_observed", obs);
        dbg_print_observation(obs);
        loc = update_locator(loc, map, obs);
        size_t maxlocs[WIDTH * HEIGHT];
        size_t maxprob = 0, maxlocn = 0;
        // also log probabilities to json
        out_json << "\"probability\":[";
        for (point lpt = {{0}}; lpt.p[1] < HEIGHT; next_point(lpt)) {
            uint32_t prob = loc.probability[point_index(lpt)];
            if (lpt.p[0] != 0 || lpt.p[1] != 0) out_json << ",";
            out_json << prob;
            if (prob > maxprob) {
                maxlocn = 0;
                maxlocs[maxlocn++] = point_index(lpt);
                maxprob = prob;
            } else if (prob == maxprob) {
                maxlocs[maxlocn++] = point_index(lpt);
            }
        }
        out_json << "]";
        // print summary
        std::printf("max probability: %.12f\n", maxprob / (double)((uint64_t)1 << 32));
        std::printf("occurs in %llu locations:\n", maxlocn);
        if (maxlocn > 5) maxlocn = 5;
        bool correct = false;
        for (size_t i = 0; i < maxlocn; i++) {
            point p = from_index(maxlocs[i]);
            if (p == pt) correct = true;
            std::printf("  (%d, %d)\n", p.p[0], p.p[1]);
        }
        if (!correct) {
            std::printf("|||||> FAILURE!\n");
        }
        std::printf("||> END OF MOVEMENT %llu\n", index + 1);
        std::fflush(stdout);
        out_json << "}";
    }
    out_json << "]}";
    return 0;
}
