#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <atomic>
#include <mutex>
#include <list>
#include <map>
#include <functional> 
#include <cctype>
#include <locale>

#include <sys/types.h>
#include <sys/stat.h>

#pragma warning(push)
#pragma warning(disable:4996)
#pragma warning(disable:4267)

#include "lasreader.hpp"
#include "laswriter.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#define BUFFERSIZE 4096
#include "b64/encode.h"


#include "civetweb/civetweb.h"

#pragma warning(disable:4244)

#include "amf-cpp/amf.hpp"
#include "amf-cpp/serializer.hpp"
#include "amf-cpp/deserializer.hpp"
#include "amf-cpp/types/amfdouble.hpp"
#include "amf-cpp/types/amfinteger.hpp"
#include "amf-cpp/types/amfvector.hpp"

#pragma warning(pop)

#include "nanoflann.hpp"

#include "ujson/ujson.hpp"

#include "DBFEngine/dbf.h"


#define _USE_MATH_DEFINES
#include <math.h>

#include <Eigen/Core>


#ifdef _WIN32
#include <direct.h>
#include "Winsock2.h"
#endif

/*
#include "7zip/LzmaLib.h"
#include "7zip/LzmaEnc.h"
*/

#define NO_CACHE \
    "Cache-Control: no-cache, no-store, must-revalidate, max-age=0\r\n" \
    "Pragma: no-cache\r\n" \
    "Expires: 0\r\n"

using namespace nanoflann;

#define NO_SSL


#ifdef NO_SSL
#ifdef USE_IPV6
#define PORT "[::]:8888"
#else
#define PORT "8888"
#endif
#else
#ifdef USE_IPV6
#define PORT "[::]:8888r,[::]:8843s,8884"
#else
#define PORT "8888r,8843s"
#endif
#endif
int exitNow = 0;

#define vassert(x, format, ...) if (!(x)) { printf(format, __VA_ARGS__); __debugbreak(); }

static std::atomic<int> plogLevel = { 0 };
#define plog(format, ...) printf("%*s" format "\n", plogLevel*2, " ", __VA_ARGS__)

struct PlogScope
{
    PlogScope() { plogLevel++; }
    ~PlogScope() { plogLevel--; }
};
#define plogScope() PlogScope plog_scope_struct

static const char* fishnetPath = "W:/gis/arso/fishnet/LIDAR_FISHNET_D96.dbf";

static const char* nameFormat = "%d_%d";
static const char* gkotPathFormat = "W:/gis/arso/laz/gkot/%s/D96TM/TM_%s.laz";
static const char* dof84PathFormat = "W:/gis/arso/dof84/%s/%s.png";

/*
static const char* otrPathFormat = "W:/gis/arso/laz/otr/b_35/D96TM/TMR_%d_%d.laz";
static const char* nanoflannAllPathFormat = "W:/gis/arso/nanoflann/gkot/b_35/D96TM/TM_%d_%d.kdz";
static const char* nanoflannGroundPathFormat = "W:/gis/arso/nanoflann/otr/b_35/D96TM/TM_%d_%d.kdtree";
*/

typedef double pcln;
typedef Eigen::Vector3d Vec;

// Ljubljana
//Vec origin{ 462000, 101000, 200 };

// Bled
//Vec origin{ 430350, 135776, 400 };
//Vec origin{ 431785, 136409, 400 };

// Smarna
//Vec origin{ 458937, 109675, 300 };

// Piran
//Vec origin{ 388143, 43887, 0 };

// Triglav
//Vec origin{ 410750, 137541, 2550 };
//Vec origin{ 410488, 138284, 2500 };
Vec origin{ 410488, 138284, 2000 };


enum Classification
{
    NONE                       = 0,
    UNASSIGNED                 = 1,
    GROUND                     = 2,
    VEGETATION_LOW             = 3,
    VEGETATION_MEDIUM          = 4,
    VEGETATION_HIGH            = 5,
    BUILDING                   = 6,
    LOW_POINT                  = 7,
    WATER                      = 9,
    BUILDING_ROOF_TILED_ORANGE = 70,
    BUILDING_ROOF_TILED_GRAY   = 71,
    BUILDING_ROOF_FLAT         = 75,
    GROUND_ASPHALT             = 80,
    GROUND_CONCRETE            = 81,
    SHADOW                     = 90,

    END
};

enum BlockType
{
    BLOCK_AIR           = 0x00,
    BLOCK_STONE         = 0x01,
    BLOCK_GRASS         = 0x02,
    BLOCK_DIRT          = 0x03,
    BLOCK_COBBLESTONE   = 0x04,
    BLOCK_PLANKS        = 0x05,
    BLOCK_SAPLING       = 0x06,
    BLOCK_BEDROCK       = 0x07,
    BLOCK_WATER_FLOWING = 0x08,
    BLOCK_WATER         = 0x09,
    BLOCK_LAVA_FLOWING  = 0x0A,
    BLOCK_LAVA          = 0x0B,
    BLOCK_SAND          = 0x0C,
    BLOCK_GRAVEL        = 0x0D,
    BLOCK_ORE_GOLD      = 0x0E,
    BLOCK_ORE_IRON      = 0x0F,
    BLOCK_ORE_COAL      = 0x10,
    BLOCK_LOG           = 0x11,

    BLOCK_LEAVES                = 0x012,
    BLOCK_LEAVES_OAK            = 0x012,
    BLOCK_LEAVES_SPRUCE         = 0x112,
    BLOCK_LEAVES_BIRCH          = 0x212,
    BLOCK_LEAVES_JUNGLE         = 0x312,
    BLOCK_LEAVES_OAK_NDECAY     = 0x412,
    BLOCK_LEAVES_SPRUCE_NDECAY  = 0x512,
    BLOCK_LEAVES_BIRCH_NDECAY   = 0x612,
    BLOCK_LEAVES_JUNGLE_NDECAY  = 0x712,
    BLOCK_LEAVES_OAK_CDECAY     = 0x812,
    BLOCK_LEAVES_SPRUCE_CDECAY  = 0x912,
    BLOCK_LEAVES_BIRCH_CDECAY   = 0xa12,
    BLOCK_LEAVES_JUNGLE_CDECAY  = 0xb12,
    BLOCK_LEAVES_OAK_NCDECAY    = 0xc12,
    BLOCK_LEAVES_SPRUCE_NCDECAY = 0xd12,
    BLOCK_LEAVES_BIRCH_NCDECAY  = 0xe12,
    BLOCK_LEAVES_JUNGLE_NCDECAY = 0xf12,

    BLOCK_WOOL              = 0x023,
    BLOCK_WOOL_WHITE        = BLOCK_WOOL,
    BLOCK_WOOL_ORANGE       = 0x123,
    BLOCK_WOOL_MAGENTA      = 0x223,
    BLOCK_WOOL_LIGHTBLUE    = 0x323,
    BLOCK_WOOL_YELLOW       = 0x423,
    BLOCK_WOOL_LIME         = 0x523,
    BLOCK_WOOL_PINK         = 0x623,
    BLOCK_WOOL_GRAY         = 0x723,
    BLOCK_WOOL_LIGHTGRAY    = 0x823,
    BLOCK_WOOL_CYAN         = 0x923,
    BLOCK_WOOL_PURPLE       = 0xA23,
    BLOCK_WOOL_BLUE         = 0xB23,
    BLOCK_WOOL_BROWN        = 0xC23,
    BLOCK_WOOL_GREEN        = 0xD23,
    BLOCK_WOOL_RED          = 0xE23,
    BLOCK_WOOL_BLACK        = 0xF23,
    
    BLOCK_GOLD              = 0x029,
    BLOCK_IRON              = 0x02A,
    BLOCK_DOUBLE_STONE_SLAB = 0x02B,
    BLOCK_STONE_SLAB        = 0x02C,

    BLOCK_CLAY              = 0x052,

    BLOCK_NETHER_BRICK      = 0x070,

    BLOCK_DOUBLE_SLAB_STONE            = 0x07d,
    BLOCK_DOUBLE_SLAB_SANDSTONE        = 0x17d,
    BLOCK_DOUBLE_SLAB_WOODEN           = 0x27d,
    BLOCK_DOUBLE_SLAB_COBBLESTONE      = 0x37d,
    BLOCK_DOUBLE_SLAB_BRICKS           = 0x47d,
    BLOCK_DOUBLE_SLAB_STONE_BRICK      = 0x57d,
    BLOCK_DOUBLE_SLAB_NETHER_BRICK     = 0x67d,
    BLOCK_DOUBLE_SLAB_QUARTZ           = 0x77d,
    BLOCK_DOUBLE_SLAB_SMOOTH_STONE     = 0x87d,
    BLOCK_DOUBLE_SLAB_SMOOTH_SANDSTONE = 0x97d,
    BLOCK_DOUBLE_SLAB_TILE_QUARTZ      = 0xf7d,

    BLOCK_QUARTZ = 0x09C,
    

};

static const std::list<std::pair<unsigned int, Classification>> classificationMap = {
    { 0x1a3f40, Classification::WATER },
    { 0x404943, Classification::WATER }, // Green
    { 0x5d7846, Classification::VEGETATION_HIGH },
    { 0x35502e, Classification::VEGETATION_HIGH },
    { 0x262127, Classification::VEGETATION_HIGH }, // Bluish
    { 0x5f7056, Classification::VEGETATION_LOW },
    { 0x567152, Classification::VEGETATION_LOW },
    { 0x636e51, Classification::VEGETATION_LOW },
    { 0x435e3d, Classification::VEGETATION_LOW },
    { 0x334a2f, Classification::VEGETATION_LOW },
    { 0xc7a08c, Classification::BUILDING_ROOF_TILED_ORANGE },
    { 0x9d695d, Classification::BUILDING_ROOF_TILED_ORANGE },
    { 0x725251, Classification::BUILDING_ROOF_TILED_ORANGE },
    { 0xc39583, Classification::BUILDING_ROOF_TILED_ORANGE },
    { 0xc1937f, Classification::BUILDING_ROOF_TILED_ORANGE },
    { 0xae7264, Classification::BUILDING_ROOF_TILED_ORANGE },
    { 0xb68978, Classification::BUILDING_ROOF_TILED_ORANGE },
    { 0xd0a794, Classification::BUILDING_ROOF_TILED_ORANGE },
    { 0xbda89d, Classification::BUILDING_ROOF_TILED_ORANGE },
    { 0x5f666b, Classification::BUILDING_ROOF_TILED_GRAY },
    { 0x858889, Classification::BUILDING_ROOF_TILED_GRAY },
    { 0x787170, Classification::BUILDING_ROOF_TILED_GRAY },
    { 0x8a8787, Classification::BUILDING_ROOF_FLAT },
    { 0xa7a3a1, Classification::BUILDING_ROOF_FLAT },
    { 0xb2aeb0, Classification::BUILDING_ROOF_FLAT },
    { 0xdedcdc, Classification::BUILDING_ROOF_FLAT },
    { 0x949293, Classification::GROUND_ASPHALT },
    { 0x858586, Classification::GROUND_ASPHALT },
    { 0x969392, Classification::GROUND_ASPHALT },
    { 0x818081, Classification::GROUND_ASPHALT },
    { 0x858586, Classification::GROUND_ASPHALT },
    { 0xa3a1a2, Classification::GROUND_ASPHALT },
    { 0xaaa6a5, Classification::GROUND_ASPHALT },
    { 0xcac5c7, Classification::GROUND_CONCRETE },
    { 0xd0cacb, Classification::GROUND_CONCRETE }, 
    { 0xbfbbba, Classification::GROUND_CONCRETE },
    // Shadows
    //*
    { 0x040814, Classification::VEGETATION_HIGH }, // Trees
    { 0x273649, Classification::GROUND_CONCRETE }, // Concrete
    { 0x253748, Classification::GROUND_CONCRETE }, // Concrete
    { 0x38414e, Classification::GROUND_ASPHALT }, // Asphalt
    { 0x1b2b3e, Classification::GROUND_ASPHALT }, // Asphalt
    { 0x0f1e30, Classification::VEGETATION_LOW }, // Grass
    { 0x060b25, Classification::WATER }, // Water
    { 0x0e2231, Classification::WATER }, // Water
    { 0x132e38, Classification::WATER }, // Water
    //*/
    /*
    { 0x040814, Classification::SHADOW }, // Trees
    { 0x273649, Classification::SHADOW }, // Concrete
    { 0x253748, Classification::SHADOW }, // Concrete
    { 0x38414e, Classification::SHADOW }, // Asphalt
    { 0x1b2b3e, Classification::SHADOW }, // Asphalt
    { 0x0f1e30, Classification::SHADOW }, // Grass
    { 0x060b25, Classification::SHADOW }, // Water
    { 0x0e2231, Classification::SHADOW }, // Water
    { 0x132e38, Classification::SHADOW }, // Water
    */
    //{ 0x, Classification::SHADOW }, // 
};

enum ClassificationFlags {
    DEFAULT  = 0,
    LARGE = 1
};


static const std::map<Classification, const char*> classificationNames = {
    { Classification::NONE,                         "none" },
    { Classification::UNASSIGNED,                   "unassigned" },
    { Classification::GROUND,                       "ground" },
    { Classification::VEGETATION_LOW,               "veg. low" },
    { Classification::VEGETATION_MEDIUM,            "veg. med" },
    { Classification::VEGETATION_HIGH,              "veg. high" },
    { Classification::BUILDING,                     "building" },
    { Classification::LOW_POINT,                    "low point" },
    { Classification::WATER,                        "water" },
    { Classification::BUILDING_ROOF_TILED_ORANGE,   "roof orange" },
    { Classification::BUILDING_ROOF_TILED_GRAY,     "roof gray" },
    { Classification::BUILDING_ROOF_FLAT,           "roof flat" },
    { Classification::GROUND_ASPHALT,               "asphalt" },
    { Classification::GROUND_CONCRETE,              "concrete" },
    { Classification::SHADOW,                       "shadow" },
};

static const std::map<Classification, int> classificationFlags = {
    { Classification::WATER, ClassificationFlags::LARGE }
};

static const std::map<Classification, std::list<Classification>> classificationSpec = {

    { Classification::GROUND, {
        Classification::WATER,
        Classification::VEGETATION_LOW,
        Classification::GROUND_ASPHALT,
        Classification::GROUND_CONCRETE
    } },

    { Classification::VEGETATION_LOW, {
        Classification::WATER,
        Classification::VEGETATION_LOW,
        Classification::GROUND_ASPHALT,
        Classification::GROUND_CONCRETE
    } },

    { Classification::VEGETATION_MEDIUM, {
        Classification::VEGETATION_MEDIUM
    } },

    { Classification::VEGETATION_HIGH, {
        Classification::VEGETATION_HIGH
    } },

    { Classification::BUILDING, {
        Classification::BUILDING_ROOF_TILED_ORANGE,
        Classification::BUILDING_ROOF_TILED_GRAY,
        Classification::BUILDING_ROOF_FLAT,
    } },

    { Classification::LOW_POINT, {
        Classification::LOW_POINT
    } },

};

struct ClassificationFilter {
    /**
     *  If target is NONE, then the closest point classification is taken
     */
    Classification target;
    std::list<Classification> sources;
    pcln radius;

    /**
     *    1: transform if at least 100% of points in `radius` around the block are target points
     *  0.5: transform if at least 50% of points in `radius` around the block are target points
     * 0.25: transform if at least 25% of points in `radius` around the block are target points
     *    0: transform if target and source points in `radius` around the block are at least equal
     * -0.5: transform if less than 50% of points in `radius` around the block are source points
     *   -1: transform if less than 100% of points in `radius` around the block are source points
     */
    pcln thresholdRatio;

    std::string name;

    ClassificationFilter(
        Classification target,
        std::list<Classification> sources,
        pcln radius,
        pcln thresholdRatio
        ) :
        target(target),
        sources(sources),
        radius(radius),
        thresholdRatio(thresholdRatio)
    {
        std::ostringstream stream;
        if (classificationNames.find(target) == classificationNames.end()) {
            stream << "filter id " << target;
        } else {
            stream << "filter " << classificationNames.at(target);
        }
        name = stream.str();
    }
};

static const std::list<ClassificationFilter> classificationFilters = {
    //*
    //*/

    {
        Classification::NONE,{
            Classification::UNASSIGNED
        }, M_SQRT2, -1
    },
    //*
    {
        Classification::GROUND_ASPHALT, {
            Classification::VEGETATION_LOW,
            Classification::VEGETATION_MEDIUM,
            Classification::GROUND,
            Classification::GROUND_CONCRETE
        }, 2, -0.2
    },


    {
        Classification::BUILDING_ROOF_TILED_ORANGE,{
            Classification::BUILDING_ROOF_FLAT,
            Classification::BUILDING_ROOF_TILED_GRAY
        }, 3, 0.2
    },
    {
        Classification::BUILDING_ROOF_FLAT,{
            Classification::BUILDING_ROOF_TILED_ORANGE
        }, 3, -0.1
    },
    {
        Classification::BUILDING_ROOF_TILED_GRAY,{
            Classification::BUILDING_ROOF_TILED_ORANGE
        }, 3, -0.1
    },


    {
        Classification::NONE, {
            Classification::BUILDING_ROOF_FLAT
        }, 4, 0.5
    },
    {
        Classification::WATER, {
            Classification::VEGETATION_LOW,
            Classification::VEGETATION_MEDIUM,
            Classification::GROUND_ASPHALT,
            Classification::GROUND_CONCRETE
        }, 6, 0.1
    },
    /*
    {
        Classification::NONE, {
            Classification::WATER
        }, 6, 0.7
    },
    //*/
    //*
    //*/
};


static inline std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

static inline std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}

// trim from start
static inline std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}


class RuntimeCounter;

class RuntimeCounters {

    typedef std::list<RuntimeCounter*> RCList;
    std::mutex mutex;

public:
    RCList list;

    void add(RuntimeCounter *c);
    RuntimeCounter* get(const char *id);
    void count();

} runtimeCounters;

enum RuntimeCounterType {
    STAT,
    EXEC_TIME
};

class Timer;

typedef unsigned long Count;
class RuntimeCounter {
    std::atomic<Count> count;

public:
    RuntimeCounterType type;
    const char *id;
    const char *name;

    Timer *timer;

    RuntimeCounter(RuntimeCounterType type, const char *id, const char *name)
        : type(type), id(id), name(name), timer(nullptr), count(0) {
        runtimeCounters.add(this);
    }

    Count operator++() { return ++count; }
    Count operator+=(Count c) { return count += c; }

    Count reset() {
        Count c = count;
        count = 0;
        return c;
    }

    void addTimer();
};


void RuntimeCounters::add(RuntimeCounter *c) {
    std::lock_guard<std::mutex> lock(mutex);
    for (auto &iter : list) { assert(strcmp(iter->id, c->id) != 0); }
    list.push_back(c);
}

RuntimeCounter* RuntimeCounters::get(const char *id) {
    std::lock_guard<std::mutex> lock(mutex);
    for (auto &iter : list) { if (strcmp(iter->id, id) == 0) return iter; }
    return nullptr;
}


#define ADD_COUNTER(id, name) RuntimeCounter id (RuntimeCounterType::STAT, #id, name);

ujson::value to_json(RuntimeCounters const &rcs) {
    auto arr = ujson::array();
    for (auto &iter : rcs.list) {
        arr.push_back(
            ujson::object {
                { "type", iter->type },
                { "id", iter->id },
                { "name", iter->name },
                { "value", static_cast<double>(iter->reset()) }
            }
        );
    }
    return arr;
}


ADD_COUNTER(boxesSent, "Boxes sent");
ADD_COUNTER(boxesCreated, "Boxes created");
ADD_COUNTER(pointsLoaded, "Points loaded");
ADD_COUNTER(requestsServed, "Requests served");



#if __cplusplus < 201103L && (!defined(_MSC_VER) || _MSC_VER < 1700)
#error Timer requires C++11
#elif 1
#include <chrono>
class Timer {
    typedef std::chrono::high_resolution_clock clock;
    typedef std::chrono::nanoseconds ns;

    std::mutex mutex;

    clock::time_point start;
    unsigned long counted;

    const char *name;
    bool printed;
    //bool counted;
    RuntimeCounter *counter;

public:
    Timer(const char *name)
    {
        this->name = name;
        tick();
        counter = runtimeCounters.get(name);
        if (!counter) {
            counter = new RuntimeCounter(RuntimeCounterType::EXEC_TIME, name, name);
        }
        counter->timer = this;
    }
    ~Timer()
    {
        count();
        counter->timer = nullptr;
    }

    unsigned long getCountedMicro()
    {
        return counted;
    }

    bool count()
    {
        /*
        if (counted) return false;
        *counter += (unsigned long)(tock().count() / 1000L);
        counted = true;
        */
        if (counter == nullptr) return false;
        unsigned long c = tock().count() / 1000L;
        *counter += c - counted;
        counted = c;
        return true;
    }

    void tick()
    {
        printed = false;
        counted = 0;
        start = clock::now();
    }
    ns tock() const
    {
        return std::chrono::duration_cast<ns>(clock::now() - start);
    }
    void print()
    {
        long long c = tock().count();
        printed = true;
        const char *unit;
        if (c < 1000) {
            unit = "ns";
        }
        else if (c < 1000000l) {
            c /= 1000l;
            unit = "us";
        }
        else {
            c /= 1000000l;
            unit = "ms";
        }
        plog("%s %*s%ld%s", name, (int)(20 - strlen(name)), " ", c, unit);
    }
};
#elif 0
class Timer {
    static HANDLE process;
    typedef ULARGE_INTEGER timestamp;

    timestamp start;
    const char *name;
    bool printed;
    bool counted;
    RuntimeCounter *counter;

public:
    Timer(const char *name)
    {
        this->name = name;
        counter = runtimeCounters.get(name);
        if (!counter) counter = new RuntimeCounter(RuntimeCounterType::EXEC_TIME, name, name);
        tick();
    }
    ~Timer()
    {
        count();
    }

    timestamp getCPUTime()
    {
        timestamp now, sys, user;

        GetProcessTimes(process, reinterpret_cast<FILETIME*>(&time), reinterpret_cast<FILETIME*>(&time), reinterpret_cast<FILETIME*>(&sys), reinterpret_cast<FILETIME*>(&user));

    }

    bool count()
    {
        if (counted) return false;
        *counter += (unsigned long)(tock().count() / 1000L);
        counted = true;
        return true;
    }

    void tick()
    {
        printed = counted = false;
        start = clock::now();
    }
    ns tock() const
    {
        return std::chrono::duration_cast<ns>(clock::now() - start);
    }
    void print()
    {
        long long c = tock().count();
        printed = true;
        const char *unit;
        if (c < 1000) {
            unit = "ns";
        }
        else if (c < 1000000l) {
            c /= 1000l;
            unit = "us";
        }
        else {
            c /= 1000000l;
            unit = "ms";
        }
        plog("%s %*s%ld%s", name, (int)(20 - strlen(name)), " ", c, unit);
    }
};
#endif

void RuntimeCounter::addTimer() {
    if (timer) timer->count();
}

void RuntimeCounters::count() {
    std::lock_guard<std::mutex> lock(mutex);
    for (auto &iter : list) { iter->addTimer(); } //if (iter->timer != nullptr) iter->timer->count(); }
}



/*
class Mutex {
    CRITICAL_SECTION mutex;

public:
    Mutex() {
        InitializeCriticalSection(&mutex);
    }

    ~Mutex() {
        DeleteCriticalSection(&mutex);
    }

    void lock() {
        EnterCriticalSection(&mutex);
    }

    void unlock() {
        LeaveCriticalSection(&mutex);
    }
};
*/

struct MapImage {
    void *data;
    int size;
    int width;
    int height;

    MapImage() : data(nullptr), size(0), width(0), height(0) {}
    ~MapImage() {
        if (data) {
            free(data);
            data = nullptr;
        }
    }
};

void rgbToComponents(unsigned int color, int &r, int &g, int &b)
{
    r = (color >> 16) & 0xFF;
    g = (color >> 8) & 0xFF;
    b = color & 0xFF;
}

void writeToBuffer(void *context, void *data, int size) {
    MapImage *img = static_cast<MapImage*>(context);
    img->data = malloc(size);
    vassert(img->data, "Unable to allocate memory for buffer");
    img->size = size;
    memcpy(img->data, data, size);
}

void parseParamLong(const char *name, long &v, char *param, char *equals)
{
    if (strncmp(name, param, (equals - param)) == 0) v = strtol(equals + 1, NULL, 10);
}

void parseParamDef(const char *name, bool &v, char *param)
{
    if (strcmp(name, param) == 0) v = true;
}

long getParamLong(const char *queryString, size_t queryLength, const char *name)
{
    char param[20];
    int ret = mg_get_var(queryString, queryLength, name, param, sizeof(param));
    return ret < 0 ? 0 : strtol(param, NULL, 10);
}

bool getParamBool(const char *queryString, size_t queryLength, const char *name)
{
    char param[20];
    int ret = mg_get_var(queryString, queryLength, name, param, sizeof(param));
    return ret < 0 || (ret > 0 && strncmp(param, "1", 1) != 0 && strncmp(param, "true", 4) != 0) ? false : true;
}

#define debugPrint(...) if (debug) mg_printf(conn, __VA_ARGS__)
#if 1
#define debug(format, ...) plog(format, __VA_ARGS__)
#define dtimer(name) Timer timer(name)
#define dtimerInit(var_id, name) Timer timer_ ## var_id {name}
#define dtimerStart(var_id) timer_ ## var_id .tick()
#define dtimerStop(var_id) timer_ ## var_id .count()
#else
#define debug(format, ...) 
#define dtimer(name)
#define dtimerInit(name)
#define dtimerStart(name)
#define dtimerStop(name)
#endif

void printArray(struct mg_connection *conn, unsigned char *data, size_t size, size_t cols = 32)
{
    mg_printf(conn, "\n\n        ");
    for (size_t i = 0; i < cols; i++) {
        mg_printf(conn, "%2d ", (int)i);
    }
    mg_printf(conn, "\n\n");
    for (size_t i = 0; i < size; i++) {
        if (i % cols == 0) mg_printf(conn, "\n%6d  ", (int)i);
        mg_printf(conn, "%2x ", data[i]);
    }
    mg_printf(conn, "\n\n");
}

bool startsWith(const char *str, const char *pre)
{
	size_t lenpre = strlen(pre),
		lenstr = strlen(str);
	return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

bool endsWith(const char *str, const char *post)
{
	size_t lenpost = strlen(post),
		lenstr = strlen(str);
	return lenstr < lenpost ? false : strncmp(str + lenstr - lenpost, post, lenpost) == 0;
}

bool mkdirp(const char *path)
{
    std::string copy = path;
    char *p = const_cast<char*>(copy.c_str());
    while (*p) {
        while (*p && *p != '/') p++;
        if (!*p) break;
        *p = 0;
        int ret = mkdir(copy.c_str());
        if (ret != 0 && errno != EEXIST) return false;
        *p = '/';
        p++;
    }
    return true;
}

/*
static void tprintf(char str[], const char* format, ...)
{
    va_list ap;
    va_start(ap, format);
    size_t len = sizeof(*str)-1;
    int ret = vsnprintf(str, len, format, ap);
    if (ret < 0) {
        va_end(ap);
        return;
    }
    if (ret >= len) {
        str[len] = 0;
    }
    else {
        str[ret] = 0;
    }
    va_end(ap);
}
*/

int
vasprintf(char **strp, const char *fmt, va_list ap)
{
    va_list ap1;
    size_t size;
    char *buffer;

    va_copy(ap1, ap);
    size = vsnprintf(NULL, 0, fmt, ap1) + 1;
    va_end(ap1);
    buffer = (char*)calloc(1, size);

    if (!buffer)
        return -1;

    *strp = buffer;

    return vsnprintf(buffer, size, fmt, ap);
}

int
asprintf(char **strp, const char *fmt, ...)
{
    int error;
    va_list ap;

    va_start(ap, fmt);
    error = vasprintf(strp, fmt, ap);
    va_end(ap);

    return error;
}

void pushInt(amf::v8 &data, int v)
{
    data.push_back((v >> 24) & 0xFF);
    data.push_back((v >> 16) & 0xFF);
    data.push_back((v >> 8) & 0xFF);
    data.push_back(v & 0xFF);
}

void writeInt(amf::u8 *p, int v)
{
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >> 8) & 0xFF;
    p[3] = v & 0xFF;
}

int readInt(const amf::u8 *p)
{
    return (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
}


class Fishnet
{

    DBF dbf;

    int records;
    int fields;

    std::string blockNotAvailable{"b_NA"};
    std::map<std::string, std::string> nameToBlock;

    int getFieldIndexFromName(const char *name) {
        for (size_t i = 0; i < fields; i++)
        {
            if (dbf.GetFieldName(i) == name) return i;
        }
        return -1;
    }

public:
    Fishnet() {}

    bool load(const char *path) {
        int ret = dbf.open(path);
        if (ret) {
            return false;
        }
        
        records = dbf.GetNumRecords();
        fields = dbf.GetNumFields();

        int colName = getFieldIndexFromName("NAME"); vassert(colName > -1, "Column not found: NAME");
        int colBlock = getFieldIndexFromName("BLOK"); vassert(colBlock > -1, "Column not found: BLOK");
        
        for (size_t i = 0; i < records; i++) {
            dbf.loadRec(i);
            std::string name = dbf.readField(colName); rtrim(name);
            std::string block = dbf.readField(colBlock); rtrim(block);
            nameToBlock.insert(std::make_pair(name, block));
        }

        dbf.close();

        return true;
    }

    const std::string& getBlockFromName(std::string name) {
        auto it = nameToBlock.find(name);
        if (it == nameToBlock.end()) return blockNotAvailable;
        return it->second;
    }

};

Fishnet fishnet;


struct Point
{
    double        x, y, z;
    unsigned char classification;
};


class PointCloud
{
    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<pcln, PointCloud>, // Distance
        PointCloud, // Dataset
        3 // Dimensions
    > KDTree;

    std::mutex mutex;

public:
    std::vector<Point> pts;
    KDTree *tree;

    PointCloud(int maxLeaf) : tree(nullptr) {
        tree = new KDTree(3, *this, KDTreeSingleIndexAdaptorParams(maxLeaf));
    }

    ~PointCloud() {
        if (tree) delete tree;
        tree = nullptr;
    }

    void addPoint(Point &point)
    {
        pts.push_back(point);
    }

    Point& getPoint(size_t index)
    {
        return pts[index];
    }

    inline size_t getPointNum() const
    {
        return pts.size();
    }

    void build()
    {
        tree->buildIndex();
    }

    void findRadius(pcln *center, RadiusResultSet<pcln, size_t> &results)
    {
        std::lock_guard<std::mutex> lock(mutex);
        tree->findNeighbors(results, center, nanoflann::SearchParams(32, 0, false));
    }

    bool findNearest(pcln *center, size_t &ret_index, pcln &out_dist_sqr)
    {
        std::lock_guard<std::mutex> lock(mutex);
        const int num_results = 1;
        KNNResultSet<pcln> result(num_results);
        result.init(&ret_index, &out_dist_sqr);
        return tree->findNeighbors(result, center, nanoflann::SearchParams());
    }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const
    {
        return getPointNum();
    }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline pcln kdtree_distance(const pcln *p1, const size_t idx_p2, size_t /*size*/) const
    {
        const Point &p = pts[idx_p2];
        const pcln d0 = p1[0] - p.x;
        const pcln d1 = p1[1] - p.y;
        const pcln d2 = p1[2] - p.z;
        return d0*d0 + d1*d1 + d2*d2;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline pcln kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim == 0) return pts[idx].x;
        else if (dim == 1) return pts[idx].y;
        else return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& bb) const {
        return false;
    }

    template <class BBOX>
    void kdtree_set_bbox(BBOX& bb) {}

};

class PointCloudIO
{
    std::mutex mutex;
    LASreader *reader;
    bool missing;

    inline bool readPoint(Point &p)
    {
        if (!reader->read_point()) return false;

        LASpoint *point = &reader->point;
        p.x = point->get_x();
        p.y = point->get_y();
        p.z = point->get_z();
        p.classification = point->classification;

        return true;
    }

public:
    PointCloudIO() : reader(nullptr) {}

    ~PointCloudIO()
    {
        close();
    }
    
    void open(const char *path)
    {
        LASreadOpener opener = LASreadOpener();
        opener.set_file_name(path);
        reader = opener.open();

        if (!reader) plog("Unable to open %s", path);
    }

    void close()
    {
        if (reader) delete reader;
        reader = nullptr;
    }

    void load(PointCloud *all, PointCloud *ground, double min_x = NAN, double min_y = NAN, double max_x = NAN, double max_y = NAN)
    {
        std::lock_guard<std::mutex> lock(mutex);

        if (reader == nullptr) return;

        vassert(reader->inside_none(), "Unable to reset LASreader bounds");
        if (!isnan(min_x) && !isnan(min_y) && !isnan(max_x) && !isnan(max_y)) {
            vassert(reader->inside_rectangle(min_x, min_y, max_x, max_y), "Unable to set LASreader bounds");
        }
        vassert(reader->seek(0), "Unable to seek to start");
        
        Point p;
        while (readPoint(p)) {
            all->addPoint(p);
            if (p.classification == Classification::GROUND) ground->addPoint(p);
        }
    }
};

/*
SRes treeProgress(void *p, UInt64 inSize, UInt64 outSize)
{
    // Update progress bar.
    return SZ_OK;
}
static ICompressProgress g_ProgressCallback = { &treeProgress };

static void * AllocForLzma(void *p, size_t size) { return malloc(size); }
static void FreeForLzma(void *p, void *address) { free(address); }
static ISzAlloc SzAllocForLzma = { &AllocForLzma, &FreeForLzma };


class TreeIndex {

    const char *path;
    FILE *file;

    size_t readPos;
    std::vector<unsigned char> inputArray;
    std::vector<unsigned char> outputArray;

public:
    bool writing = false;

    TreeIndex() : path(nullptr), file(nullptr) {}
    ~TreeIndex() { cleanup(); }

    bool readChecked(void * ptr, size_t size, const char *errorMessage) {
        size_t readb = fread(ptr, 1, size, file);
        if (readb != size) {
            plog("%s", errorMessage);
            return false;
        }
        return true;
    }

    bool readChecked(size_t *ptr, const char *errorMessage) {
        unsigned char bytes[4];
        if (!readChecked(bytes, 4, errorMessage)) return false;
        int n;
        n = readInt(bytes);
        *ptr = n;
        return true;
    }

    bool begin(const char *path) {
        cleanup();

        this->path = path;

        if (!writing) {
            // Reading
            file = fopen(path, "rbR");
            if (!file) return false;

            struct _stat st;
            int fd = _fileno(file);
            if (_fstat(fd, &st)) {
                plog("Unable to fstat file: %s", path);
                return error();
            }
            _dev_t size = st.st_size;

            Byte propsEncoded[LZMA_PROPS_SIZE];
            if (!readChecked(propsEncoded, LZMA_PROPS_SIZE, "Unable to read props")) return error();
            
            size_t uncompressedSize;
            if (!readChecked(&uncompressedSize, "Unable to read props")) return error();

            inputArray.resize(size - LZMA_PROPS_SIZE - 4);
            if (!readChecked(inputArray.data(), inputArray.size(), "Unable to read tree index")) return error();

            size_t inputSize = inputArray.size();
            outputArray.resize(uncompressedSize);
            SRes ret = LzmaUncompress(outputArray.data(), &uncompressedSize, inputArray.data(), &inputSize, propsEncoded, LZMA_PROPS_SIZE);
            
            if (ret != SZ_OK) { plog("Unable to uncompress tree index"); return error(); }

            inputArray.clear();
            readPos = 0;

            fclose(file); file = nullptr;
        }

        return true;
    }

    bool error() {
        cleanup();
        return false;
    }

    void cleanup() {
        if (file) { fclose(file); file = nullptr; }
        inputArray.clear();
        outputArray.clear();
    }

    bool write(char *mem, size_t size) {

        inputArray.insert(inputArray.end(), mem, mem + size);

        return true;
    }

    size_t read(char *mem, size_t size, size_t count) {
        if (writing) { plog("Unable to read a tree index in write mode: %s", path); return error(); }

        memcpy(mem, &outputArray[readPos], size*count);
        readPos += size*count;

        return count;
    }

    bool writeChecked(const void * ptr, size_t size, const char *errorMessage) {
        size_t written = fwrite(ptr, 1, size, file);
        if (written != size) {
            plog("%s", errorMessage);
            return false;
        }
        return true;
    }

    bool writeChecked(size_t n, const char *errorMessage) {
        unsigned char bytes[4];
        writeInt(bytes, n);
        return writeChecked(bytes, 4, errorMessage);
    }

    bool end() {

        if (writing) {
            outputArray.resize(inputArray.size());
            SizeT outputLen = outputArray.size();


            CLzmaEncProps props;
            LzmaEncProps_Init(&props);

            props.algo = 0;

            Byte propsEncoded[LZMA_PROPS_SIZE];
            SizeT propsSize;

            plog("Compressing tree index");

            SRes res = LzmaEncode(
                outputArray.data(),
                &outputLen,
                inputArray.data(),
                inputArray.size(),
                &props,
                propsEncoded,
                &propsSize,
                props.writeEndMark,
                &g_ProgressCallback,
                &SzAllocForLzma,
                &SzAllocForLzma
            );

            plog("Saving tree index");
            
            if (propsSize != LZMA_PROPS_SIZE) { plog("Invalid props size: %d", (int)propsSize); return error(); }
            if (res != SZ_OK) { plog("Unable to compress tree index"); return error(); }

            file = fopen(path, "wb");
            if (!file) { plog("Unable to open file for writing: %s", path); return error(); }

            if (!writeChecked(propsEncoded, propsSize, "Unable to write header props")) return error();
            if (!writeChecked(inputArray.size(), "Unable to write header props")) return error();
            if (!writeChecked(outputArray.data(), outputLen, "Unable to write compressed bytes")) return error();

            fclose(file);
        }

        cleanup();

        return true;
    }

};


static void treeWriter(const void * ptr, size_t size, size_t count, void *userdata) {
    TreeIndex *ti = static_cast<TreeIndex*>(userdata);
    ti->write((char*)ptr, size*count);
}

static size_t treeReader(const void * ptr, size_t size, size_t count, void *userdata) {
    TreeIndex *ti = static_cast<TreeIndex*>(userdata);
    return ti->read((char*)ptr, size, count);
}
*/

template <typename num_t>
class PointSearch {
    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<num_t, PointCloud>, // Distance
        PointCloud, // Dataset
        3 // Dimensions
    > KDTree;

    std::mutex mutex;

    KDTree *tree;
    int maxLeaf;

    std::string treePath;

public:
    PointCloud cloud;

    PointSearch(const char* pclPath, const char* treePath, int maxLeaf) : cloud(PointCloud(pclPath)), tree(nullptr) {
        this->maxLeaf = maxLeaf;
        this->treePath = treePath;
    }

    ~PointSearch() {
        deleteTree();
    }

    void deleteTree()
    {
        if (tree) delete tree;
        tree = nullptr;
    }

    void loadTree()
    {
        plogScope();

        deleteTree();

        tree = new KDTree(3, cloud, KDTreeSingleIndexAdaptorParams(maxLeaf));

        char treeAllTempPath[1024];
        TreeIndex ti;

        if (!ti.begin(treePath.c_str())) {
            // Index missing

            mkdirp(treePath.c_str());

            sprintf_s(treeAllTempPath, "%s.temp", treePath.c_str());

            plog("Processing %d points", cloud.kdtree_get_point_count());
            cloud.preload();

            plog("Building tree index");

            tree->buildIndex();

            cloud.unpreload();

            plog("Serializing tree index");

            ti.writing = true;
            if (!ti.begin(treeAllTempPath)) {
                plog("Unable to begin tree index");
                return;
            }

            tree->saveIndex(treeWriter, &ti);
        } else {
            // Index exists
            tree->loadIndex(treeReader, &ti);
        }

        if (!ti.end()) {
            plog("Unable to end tree index");
            return;
        }

        if (ti.writing) {
            int ret = rename(treeAllTempPath, treePath.c_str());
            if (ret) plog("Unable to save index, file already exists: %s", treePath.c_str());
        }
    }

    void findRadius(num_t *center, RadiusResultSet<num_t, size_t> &results)
    {
        std::lock_guard<std::mutex> lock(mutex);
        tree->findNeighbors(results, center, nanoflann::SearchParams(32, 0, false));
    }

    bool findNearest(num_t *center, size_t &ret_index, num_t &out_dist_sqr)
    {
        std::lock_guard<std::mutex> lock(mutex);
        const int num_results = 1;
        KNNResultSet<num_t> result(num_results);
        result.init(&ret_index, &out_dist_sqr);
        return tree->findNeighbors(result, center, nanoflann::SearchParams());
    }

};

#include <condition_variable>

class MapCloud;

struct ClassificationQuery {
    double x;
    double y;
	MapCloud** corners;
    Classification lidar;
    int flags;

	ClassificationQuery() : x(NAN), y(NAN), corners(nullptr), lidar(Classification::NONE), flags(ClassificationFlags::DEFAULT) {}
};

class MapCloud {
public:
    static const int READER_NUM = 6;
    const int lat;
    const int lon;

    bool readerFree[READER_NUM];
    PointCloudIO readers[READER_NUM];
    int readersFree = READER_NUM;

    MapImage map;

protected:
    const char *lidarPath;
    const char *mapPath;

    std::mutex mutex;
    std::condition_variable cond;
public:

    MapCloud(int lat, int lon, const char *lidarPath, const char *mapPath) : 
        lat(lat), lon(lon),
        lidarPath(lidarPath),
        mapPath(mapPath)
    {
        open();
    }

    ~MapCloud() {
        close();
    }

    void open() {
        plogScope();

        {
            dtimer("mapcloud readers");
            close();
            for (int i = 0; i < READER_NUM; i++) {
                readers[i].open(lidarPath);
                readerFree[i] = true;
            }
        }

        {
            dtimer("mapcloud map image");

            int reqComp = 3;
            int retComp;
            map.data = stbi_load(mapPath, &map.width, &map.height, &retComp, reqComp);
            if (map.data == nullptr) {
                plog("Unable to open %s", mapPath);
            } else {
                assert(reqComp == retComp);
            }
        }
    }

    void close() {
        for (int i = 0; i < READER_NUM; i++) {
            readerFree[i] = false;
            readers[i].close();
        }

        if (map.data) {
            stbi_image_free(map.data);
            map.data = nullptr;
        }
    }

    void load(PointCloud *all, PointCloud *ground, double min_x = NAN, double min_y = NAN, double max_x = NAN, double max_y = NAN) {
        int readerIndex = -1;
        PointCloudIO *reader;

        {
            std::unique_lock<std::mutex> lock(mutex);
            while (readersFree == 0) cond.wait(lock);
            for (int i = 0; i < READER_NUM; i++) {
                if (!readerFree[i]) continue;
                readerIndex = i;
                readerFree[readerIndex] = false;
                reader = &readers[readerIndex];
                break;
            }
            vassert(readerIndex > -1, "Unable to find free reader");
            readersFree--;
        }

        reader->load(all, ground, min_x, min_y, max_x, max_y);

        {
            std::unique_lock<std::mutex> lock(mutex);
            readerFree[readerIndex] = true;
            reader = nullptr;
            readersFree++;
            cond.notify_one();
        }
    }

    static Classification classifyPixel(const ClassificationQuery &cq, unsigned int rgb) {
        int tr, tg, tb;

        rgbToComponents(rgb, tr, tg, tb);

        long minDist = MAXLONG;
        Classification minClassification = Classification::NONE;

        const std::list<Classification> *specializesTo = nullptr;
        auto classSpec = classificationSpec.find(cq.lidar);
        if (classSpec != classificationSpec.end()) specializesTo = &classSpec->second;

        //long distances[Classification::END] = {};
        //int counts[Classification::END] = {};

        for (auto &iter : classificationMap) {
            // Skip unspecialized points
            if (specializesTo && std::find(specializesTo->begin(), specializesTo->end(), iter.second) == specializesTo->end()) continue;

            /*
            int iterFlags = 0;
            auto foundFlags = classificationFlags.find(iter.second);
            if (foundFlags != classificationFlags.end()) iterFlags = foundFlags->second;
            */
            // Is this right?
            //if (iterFlags & !cq.flags) continue;


            int mr, mg, mb;
            rgbToComponents(iter.first, mr, mg, mb);

            int dr = mr - tr;
            int dg = mg - tg;
            int db = mb - tb;

            long dist = dr*dr + dg*dg + db*db;
            /*
            distances[iter.second] += dist;
            counts[iter.second]++;
            //*/
            //*
            if (dist < minDist) {
                minDist = dist;
                minClassification = iter.second;
            }
            //*/
        }

        /*
        for (int i = 0; i < Classification::END; i++) {
            int count = counts[i];
            if (count == 0) continue;
            long dist = distances[i] / count;
            if (dist < minDist) {
                minDist = dist;
                minClassification = static_cast<Classification>(i);
            }
        }
        */

        return minClassification;
    }

	static MapCloud* fromCorners(MapCloud** corners, double x, double y, int *mx = nullptr, int *my = nullptr) {
		int cornerNum = 4;
		int mw = 1000;
		int mh = 1000;
		for (int i = 0; i < cornerNum; i++) {
			MapCloud* mapCloud = corners[i];
			if (!mapCloud) continue;
			int mapX = (int)(x - mapCloud->lat*mw);
			int mapY = (int)(1000 - 1 - (y - mapCloud->lon*mh));
			if (mapX >= 0 && mapY >= 0 && mapX < mw && mapY < mh) {
				if (mx) *mx = mapX;
				if (my) *my = mapY;
				return mapCloud;
			}
		}
		return nullptr;
	}

	static unsigned int getMapColor(MapCloud** corners, double x, double y) {
		int mx;
		int my;
		MapCloud *mapCloud = fromCorners(corners, x, y, &mx, &my);
		if (!mapCloud) return -1;
		return mapCloud->getMapPointColor(mx, my);
	}

	unsigned int getMapPointColor(int mx, int my) const {
		if (mx < 0) mx = 0;
		if (my < 0) my = 0;
		if (mx >= map.width - 1) mx = map.width - 1;
		if (my >= map.height - 1) my = map.height - 1;

        if (map.data == nullptr) return 0;

		unsigned char* md = static_cast<unsigned char*>(map.data);

		int mapIndex = (mx + my * map.width) * 3;
		return (md[mapIndex + 0] << 16) | (md[mapIndex + 1] << 8) | md[mapIndex + 2];
	}

    Classification getMapPointClassification(const ClassificationQuery &cq, int mx, int my) {
		unsigned int pixel = getMapPointColor(mx, my);
        return classifyPixel(cq, pixel);
    }

    static Classification getSpecializedClassification(const ClassificationQuery &cq, MapCloud** cloudOut = nullptr, int* mapX = nullptr, int* mapY = nullptr) {

		vassert(!isnan(cq.x) && !isnan(cq.y), "Classification query invalid coordinates");
		vassert(cq.corners != nullptr, "Classification query corners missing");

		int mx;
		int my;
		MapCloud *mapCloud = fromCorners(cq.corners, cq.x, cq.y, &mx, &my);
		if (cloudOut) *cloudOut = mapCloud;
		if (mapX) *mapX = mx;
		if (mapY) *mapY = my;
		if (!mapCloud) return Classification::NONE;

        Classification center = mapCloud->getMapPointClassification(cq, mx, my);

        int flags = 0;
        auto foundFlags = classificationFlags.find(center);
        if (foundFlags != classificationFlags.end()) flags = foundFlags->second;
        
        const int step = (flags & ClassificationFlags::LARGE) ? 3 : 1;
        const int range = (flags & ClassificationFlags::LARGE) ? 2 : 0;

        if (range > 0) {
            int counts[Classification::END] = {};
            for (int oy = -range; oy <= range; oy++) {
                for (int ox = -range; ox <= range; ox++) {
                    if (ox == 0 && oy == 0) continue;
                    int omx = mx + ox*step;
                    int omy = my + oy*step;
					Classification c = mapCloud->getMapPointClassification(cq, omx, omy);
                    counts[c]++;
                }
            }

            int maxCount = 0;
            Classification maxClassification;
            for (int i = 0; i < Classification::END; i++) {
                int count = counts[i];
                if (count > maxCount) {
                    maxCount = count;
                    maxClassification = static_cast<Classification>(i);
                }
            }

            return maxCount > 1 ? maxClassification : center;
        }

        return center;
    }

};

template <typename T>
struct SpatialHash
{
    bool initialized = false;
    unsigned int size;
    unsigned int mask;
    T* hash;

    SpatialHash(unsigned int bits)
    {
        size = 1 << bits;
        mask = size - 1;
        hash = new T[size];
    }

    ~SpatialHash()
    {
        if (!initialized) return;
        if (hash) {
            delete hash;
            hash = nullptr;
        }
    }

    T& at(int x, int y)
    {
        unsigned int hashCode;
        return at(x, y, hashCode);
    }

    T& at(int x, int y, unsigned int &hashCode)
    {
        unsigned int tmx = x & 0xFFFF;
        unsigned int tmy = y & 0xFFFF;
        tmx = (tmx | (tmx << 8)) & 0x00FF00FF;
        tmx = (tmx | (tmx << 4)) & 0x0F0F0F0F;
        tmx = (tmx | (tmx << 2)) & 0x33333333;
        tmx = (tmx | (tmx << 1)) & 0x55555555;
        tmy = (tmy | (tmy << 8)) & 0x00FF00FF;
        tmy = (tmy | (tmy << 4)) & 0x0F0F0F0F;
        tmy = (tmy | (tmy << 2)) & 0x33333333;
        tmy = (tmy | (tmy << 1)) & 0x55555555;
        unsigned int coordHash = tmx | (tmy << 1);
        hashCode = coordHash & mask;

        return hash[hashCode];
    }
};

//static SpatialHash<MapCloud<double>> mapCloudHash(2);
static std::list<MapCloud*> mapCloudList;
static std::mutex mapCloudListMutex;

class ProgressPrinter {
protected:
    int current;
    const int total = 5;

public:

    ProgressPrinter() : current(0) {}

    void progress(double p)
    {
        int v = static_cast<int>(round(p*total));
        if (v > current) {
            current = v;
            printf(".");
        }
    }
};

MapCloud* getMapCloud(int lat, int lon)
{
    /*
    reader->inside_rectangle(
    //      462000           101000
    reader->get_min_x() + 0, reader->get_min_y() + 0,
    //      462999.98999999  101999.99000000001
    reader->get_max_x() - 0, reader->get_max_y() + 0
    );
    */
    //int skip = 5;

    //n /= skip; n++;

    //n = 1000000;
    /*
    mc.cloud = new PointCloud<num_t>();
    mc.cloud->pts.resize(n);
    mc.cloud->min[0] = reader->get_min_x(); mc.cloud->max[0] = reader->get_max_x();
    mc.cloud->min[1] = reader->get_min_y(); mc.cloud->max[1] = reader->get_max_y();
    mc.cloud->min[2] = reader->get_min_z(); mc.cloud->max[2] = reader->get_max_z();
    */



    std::lock_guard<std::mutex> lock(mapCloudListMutex);
    
    for (auto element = mapCloudList.begin(); element != mapCloudList.end(); element++) {
        if ((*element)->lat == lat && (*element)->lon == lon) {
            return *element;
        }
    }

    // Not found, create a new one

    //plog("Initializing new map cloud at %d, %d", lat, lon);

    char name[512],
         allPath[1024],
         dof84Path[1024];
         //treeAllPath[1024],
         //groundPath[1024],
         //treeGroundPath[1024];

    sprintf_s(name, nameFormat, lat, lon);

    std::string block = fishnet.getBlockFromName(name);

    sprintf_s(allPath, gkotPathFormat, block.c_str(), name);
    sprintf_s(dof84Path, dof84PathFormat, block.c_str(), name);

    //sprintf_s(groundPath, otrPathFormat, lat, lon);
    //sprintf_s(treeAllPath, nanoflannAllPathFormat, lat, lon);
    //sprintf_s(treeGroundPath, nanoflannGroundPathFormat, lat, lon);

    MapCloud *mc = new MapCloud(lat, lon, allPath, dof84Path);
    mapCloudList.push_front(mc);

    //if (mc.all) delete mc.all;
    //if (mc.ground) delete mc.ground;

    //plogScope();

    //mc.all    = new PointSearch<num_t>(allPath, treeAllPath, 50);
    //mc.ground = new PointSearch<num_t>(groundPath, treeGroundPath, 20);

    //mc.all->cloud.setPointNum(n);

    /*
    if (mc.ground) delete mc.ground;
    mc.ground = new PointSearch<num_t>(20);
    */
    /*
    size_t index = 0;
    ProgressPrinter pp;
    while (index < n && reader->read_point())
    {
        LASpoint *point = &reader->point;

        Point<num_t> &cp = mc.all->cloud.getPoint(index);
        cp.x = point->get_x();
        cp.y = point->get_y();
        cp.z = point->get_z();
        cp.classification = point->classification;

        //if (point->classification == Classification::GROUND) {
        //    mc.ground->cloud.addPoint(*cp);
        //}
        
        index++;
        pp.progress((double)index / n);
    }

    reader->close();
    delete reader;

    printf("\n");
    

    */

    /*
    plog("Loading all point index (%d)", mc.all->cloud.kdtree_get_point_count());
    mc.all->loadTree();

    plog("Loading ground point index (%d)", mc.ground->cloud.kdtree_get_point_count());
    mc.ground->loadTree();
    */
    
    //printf("  Building KDTree for %d ground points\n", mc.ground->cloud.pts.size());
    //mc.ground->buildTree();

    //plog("Done");
    return mc;
}

static unsigned int classificationToBlock(unsigned int cv)
{
    /*
    switch (classification)
    {
    case 1: bid = 13; break; // Unassigned -> Gravel
    case 2: bid = 43; break; // Ground -> Double Stone Slab
    case 3:
    case 4: bid = 3; break; // Low/Medium Vegetation -> Dirt
    case 6: bid = 98; break; // Building -> Stone Bricks
    case 5: bid = 18; break; // High Vegetation -> Leaves
    case 9: bid = 9; break; // Water -> Still Water
    }
    //*/

    // uint element value
    // 0x000000FF	classification
    // 0x00000F00	blocklight

    /*
    switch (classification)
    {
    case 0:                                 bid = 0;  bd =  0; break;
    *//*
    case Classification::UNASSIGNED:        bid = 35; bd =  0; break; // Unassigned -> White Wool
    case Classification::GROUND:            bid = 35; bd = 12; break; // Ground -> Brown Wool
    case Classification::VEGETATION_LOW:    bid = 35; bd =  4; break; // Low Vegetation -> Yellow Wool
    case Classification::VEGETATION_MEDIUM: bid = 35; bd =  5; break; // Medium Vegetation -> Green Wool
    case Classification::BUILDING:          bid = 35; bd =  8; break; // Building -> Light Gray Wool
    case Classification::VEGETATION_HIGH:   bid = 35; bd = 13; break; // High Vegetation -> Green Wool
    case Classification::LOW_POINT:         bid = 35; bd = 15; break; // Low Point (Noise) -> Black Wool
    case Classification::WATER:             bid = 35; bd =  3; break; // Water -> Light Blue Wool
    *//*
    case Classification::UNASSIGNED:        bid = 35; bd = 0; break;
    case Classification::GROUND:            bid = 43; bd = 0; break;
    case Classification::VEGETATION_LOW:    bid = 43; bd = 0; break;
    case Classification::VEGETATION_MEDIUM: bid = 43; bd = 0; break;
    case Classification::BUILDING:          bid = 35; bd = 8; break;
    case Classification::VEGETATION_HIGH:   bid = 35; bd = 13; break;
    case Classification::LOW_POINT:         bid = 35; bd = 15; break;
    case Classification::WATER:             bid = 35; bd = 3; break;
    }
    */

    unsigned int bv;

    // Custom blocks
    if (cv & (1 << 31)) return cv & ~(1 << 31);
    //if (cv & (1 << 31)) return 0;

    int classification = cv & 0xFF;
    int blocklight = (cv >> 8) & 0xF;

    switch (classification)
    {
    case Classification::NONE:              bv = BLOCK_AIR; break;
    case Classification::UNASSIGNED:        bv = BLOCK_AIR; break;
    case Classification::GROUND:            bv = BLOCK_DIRT; break;
    case Classification::VEGETATION_LOW:    bv = BLOCK_GRASS; break;
    case Classification::VEGETATION_MEDIUM: bv = BLOCK_LEAVES_SPRUCE; break;
    case Classification::VEGETATION_HIGH:   bv = BLOCK_LEAVES_OAK; break;
    case Classification::BUILDING:          bv = BLOCK_WOOL_LIGHTGRAY; break;
    case Classification::LOW_POINT:         bv = BLOCK_AIR; break;
    case Classification::WATER:             bv = BLOCK_WATER; break;

    /*
    case Classification::BUILDING_ROOF_TILED_ORANGE: bv = BLOCK_DOUBLE_SLAB_BRICKS; break;
    case Classification::BUILDING_ROOF_TILED_GRAY:   bv = BLOCK_DOUBLE_SLAB_STONE_BRICK; break;
    case Classification::BUILDING_ROOF_FLAT:         bv = BLOCK_DOUBLE_SLAB_SMOOTH_STONE; break;
    */

    case Classification::BUILDING_ROOF_TILED_ORANGE: bv = BLOCK_WOOL_ORANGE; break;
    case Classification::BUILDING_ROOF_TILED_GRAY:   bv = BLOCK_WOOL_LIGHTGRAY; break;
    case Classification::BUILDING_ROOF_FLAT:         bv = BLOCK_WOOL_WHITE; break;

    case Classification::GROUND_ASPHALT:    bv = BLOCK_STONE; break;
    case Classification::GROUND_CONCRETE:   bv = BLOCK_WOOL_LIGHTGRAY; break;
    case Classification::SHADOW:            bv = BLOCK_WOOL_BLUE; break;
    default:                                bv = BLOCK_AIR; break;
    }

    //if (bv != BLOCK_DIRT) bv = BLOCK_AIR;

    // uint element value
    // 0x000000FF	block id
    // 0x00000F00	block data
    // 0x0000F000	skylight
    // 0x000F0000	blocklight
    // 0x0FF00000	? height of column (stored at y = 0)

    return (blocklight << 16) | bv;
}

#define EXPORT_BOX_DEBUG 0
#define EXPORT_BOX_BINARY 0
#define EXPORT_BOX_JSON 1

enum ExportPointCommand {
    CommandBase = 0,
    ClassificationBase = 0x100
};

struct BoxResult {
    std::mutex mutex;
    
    std::atomic<long> access;

    bool valid;

    long x, y, z;
    long sx, sy, sz;

	amf::v8 *data;

    /*
	// TODO: Put stuff below into a supplementary class and observe mem usage

	MapCloud* cornerClouds[4];
	Vec bounds_tl;
	Vec bounds_br;
	Vec bounds_min;
	Vec bounds_max;
	long bx, by, bz;

	// Retained in debug mode
	std::vector<unsigned int> blocks;
	std::vector<unsigned int> columns;
	std::vector<unsigned int> groundCols;
    */

#if EXPORT_BOX_DEBUG
    const char* path;
    FILE *file;
    bool firstLine;
    std::map<int, int> cidMap;
#endif

    BoxResult() : data(nullptr), access(0), valid(false) {};
    ~BoxResult() {
        if (data) delete data;
        data = nullptr;
    }


    void exportOpen() {
#if EXPORT_BOX_DEBUG
#if EXPORT_BOX_JSON
        path = "W:/js/data-projector/data.json";
        //sprintf_s(path, MAX_PATH - 1, "points_%d_%d_%d_%d_%d_%d.json", sx, sy, sz, bx, by, bz);
        file = fopen(path, "w");
        assert(file);
        fputs("{ \"points\": [", file);
        firstLine = true;
#elif EXPORT_BOX_BINARY
        char path[MAX_PATH] = "C:/Program Files (x86)/Epic Games/4.12/Engine/Binaries/Win64/points.bin";
        file = _fsopen(path, "wbS", _SH_DENYNO);
#endif
        vassert(file, "Unable to open file: %s", path);
        cidMap.clear();
#endif
    }

    void exportWrite(pcln x, pcln y, pcln z, int cid) {
#if EXPORT_BOX_DEBUG
        static std::mutex mutex;
        std::pair<std::map<int, int>::iterator, bool> res = cidMap.insert(std::make_pair(cid, (int)cidMap.size()));
        cid = (res.first)->second;
#if EXPORT_BOX_JSON
        fprintf(file, "%s\n{ \"x\": %f, \"y\": %f, \"z\": %f, \"cid\": %d }", firstLine ? "" : ",", x, y, z, cid);
        firstLine = false;
#elif EXPORT_BOX_BINARY
        unsigned int cmd = ExportPointCommand::ClassificationBase + cid;
        float fx = (float)x;
        float fy = (float)y;
        float fz = (float)z;
        {
            std::lock_guard<std::mutex> lock(mutex);
            fwrite(&cmd, sizeof(unsigned int), 1, file);
            fwrite(&fx, sizeof(float), 1, file);
            fwrite(&fy, sizeof(float), 1, file);
            fwrite(&fz, sizeof(float), 1, file);
        }
#endif
#endif
    }


    void exportClose() {
#if EXPORT_BOX_DEBUG
        if (!file) return;
#if EXPORT_BOX_JSON
        fputs("\n] }", file);
#endif
        fclose(file);
        file = nullptr;
        plog("Debug points written to %s", path);
#endif
    }


    void send(struct mg_connection *conn) {
        mg_printf(conn,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/x-amf\r\n"
            NO_CACHE
            "\r\n"
        );
        assert(data);
        mg_write(conn, data->data(), data->size());
    }
};

static const long boxHashAccessStart = 0xFF;
static std::atomic<long> boxHashAccess = { boxHashAccessStart };
static SpatialHash<BoxResult> boxHash(11);
static BoxResult invalidBoxResult;

static const int blockImage[9] = {
    0xFF, 0xFF, 0xFF,
    0xFF, 0x00, 0xFF,
    0xFF, 0xFF, 0xFF
};

static unsigned int blockToColor[0x1000];

static void paintImage(char *pixels, int pixelsWidth, int x, int y, const int *image, int width, int height)
{
    pixels = &pixels[x + y * pixelsWidth];
    for (int iy = 0; iy < height; iy++) {
        for (int ix = 0; ix < width; ix++) {
            int v = image[ix + iy*width];
            pixels[0] = v;
            pixels[1] = v;
            pixels[2] = v;
            pixels += 3;
        }
        pixels += pixelsWidth - width;
    }
}

static void paintRect(unsigned int *pixels, int pixelsWidth, int x, int y, unsigned int color, int width, int height)
{
    unsigned char *comps = (unsigned char*)(&pixels[x + y * pixelsWidth]);
    char a = (color >> 24) & 0xFF;
    char r = (color >> 16) & 0xFF;
    char g = (color >> 8) & 0xFF;
    char b = (color) & 0xFF;
    for (int iy = 0; iy < height; iy++) {
        for (int ix = 0; ix < width; ix++) {
            comps[ix*4 + 0] = r;
            comps[ix*4 + 1] = g;
            comps[ix*4 + 2] = b;
            comps[ix*4 + 3] = a;
        }
        comps += pixelsWidth*4;
    }
}

static void createColumnVis(unsigned int *columns, int sx, int sz, unsigned int **pixels, int &width, int &height)
{
    int sxz = sx*sz;

    int imagePadding = 4;
    int blockPadding = 1;
    int blockWidth = 10 + blockPadding;
    int blockHeight = 10 + blockPadding;

    width = imagePadding * 2 + blockWidth * sx;
    height = imagePadding * 2 + blockHeight * sz;
    *pixels = (unsigned int*)calloc(width*height, 4);

    for (int iz = 0; iz < sz; iz++) {
        for (int ix = 0; ix < sx; ix++) {
            unsigned int rx = imagePadding + ix*blockWidth;
            unsigned int ry = imagePadding + iz*blockWidth;
            int colindex = ix + iz*sx;
            int height = columns[colindex];
            height = std::min(0xFF, std::max(0, height));
            unsigned int color = (0 << 16) | (height << 8) | 0;
            color |= 0xFF000000;
            paintRect(*pixels, width, rx, ry, color, blockWidth - blockPadding, blockHeight - blockPadding);
        }
    }

}

static void createBlockVis(unsigned int *cblocks, int sx, int sy, int sz, unsigned int **pixels, int &width, int &height)
{
    int sxz = sx*sz;

    int imagePadding = 4;
    int blockPadding = 1;
    int blockWidth = 10;
    int blockHeight = 3;
    int blockLayerPadding = 2;

    width = imagePadding * 2 + blockWidth * sx;
    height = imagePadding * 2 + (sz + blockLayerPadding) * sy * blockHeight;
    *pixels = (unsigned int*)calloc(width*height, 4);
    /*
    for (int iy = 0; iy < height; iy++) {
    for (int ix = 0; ix < width; ix++) {
    (*pixels)[(ix + iy*width) * 3 + 0] = 0x00;
    (*pixels)[(ix + iy*width) * 3 + 1] = 0xFF;
    (*pixels)[(ix + iy*width) * 3 + 2] = 0x00;
    }
    }
    */
    //*
    for (int iy = 0; iy < sy; iy++) {
        for (int iz = 0; iz < sz; iz++) {
            for (int ix = 0; ix < sx; ix++) {
                int rx = imagePadding + ix * blockWidth;
                int ry = height - (imagePadding + (iz + iy*(sz + blockLayerPadding))*blockHeight);
                unsigned int bv = cblocks[ix + iz*sx + iy*sxz];
                unsigned int color = blockToColor[bv];
                if (color) color |= 0xFF000000;
                paintRect(*pixels, width, rx, ry, color, blockWidth - blockPadding, blockHeight - blockPadding);
            }
        }
    }
    //*/
}

static std::mutex encodeMutex;

static void encodeImage(MapImage *img, unsigned int *pixels, int width, int height)
{
    stbi_write_png_to_func(&writeToBuffer, img, width, height, 4, reinterpret_cast<char*>(pixels), 0);
}

static void printImage(struct mg_connection *conn, unsigned int *pixels, int width, int height)
{
    MapImage img;
    encodeImage(&img, pixels, width, height);
    std::istringstream pngIn = std::istringstream(std::string(static_cast<const char*>(img.data), img.size));
    base64::encoder encoder;
    std::ostringstream pngOut;
    encoder.encode(pngIn, pngOut);
    std::string pngDataURI = "data:image/png;base64," + pngOut.str();

    mg_printf(conn, "<img width=\"%d\" height=\"%d\" alt=\"map\" src=\"%s\" />", width, height, pngDataURI.c_str());
}

void sendLiveImageHeader(struct mg_connection *conn, int size)
{
	mg_printf(conn,
		"HTTP/1.1 200 OK\r\nContent-Type: image/png\r\n"
		NO_CACHE
		"Content-Length: %d\r\n"
		"\r\n", size);
}

void sendLiveJSONHeader(struct mg_connection *conn, int size)
{
	mg_printf(conn,
		"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
		NO_CACHE
		"Content-Length: %d\r\n"
		"\r\n", size);
}

static inline int getBlockIndex(int bx, int by, int bz, int sx, int sxz)
{
    return bx + bz*sx + by*sxz;
}

static inline void getIndexBlock(int index, int &bx, int &by, int &bz, int sx, int sz)
{
    bx = index % sx;
    bz = (index / sx) % sz;
    by = index / (sx*sz);
}

static inline int getColumnIndex(int bx, int bz, int sx)
{
	return bx + bz*sx;
}

static void renderBoxesUsed(MapImage &img)
{
    int padding = 0;

    int width = 32 + padding * 2;
    int height = width;

    int tileSize = 10;

    int pixelWidth = width*tileSize;
    int pixelHeight = height*tileSize;

    size_t pixelsLen = pixelWidth*pixelHeight;
    std::unique_ptr<unsigned int> pixels((new unsigned int[pixelsLen]()));

    for (int iy = 0; iy < height; iy++) {
        for (int ix = 0; ix < width; ix++) {
            //if (ix + iy*width >= len) break;

            BoxResult &br = boxHash.at(ix - padding, iy - padding);

            char r = 0;
            char g = 0;
            char b = 0;

            r = g = b = std::max(0, 0xFF - (int)(boxHashAccess - br.access));

            unsigned int color = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);

            color |= 0xFF000000;
            paintRect(pixels.get(), pixelWidth, ix*tileSize, iy*tileSize, color, tileSize, tileSize);
        }
    }

    encodeImage(&img, pixels.get(), pixelWidth, pixelHeight);
}

static void renderMapClouds(MapImage &img)
{
    int subtileWidth = 6;
    int subtileNum = MapCloud::READER_NUM;
    int border = 1;
    int tileSize = subtileWidth*subtileNum + border*2;
    int tileGrid = 10;

    int centerLat = 462;
    int centerLon = 101;

    int width = tileGrid;
    int height = width;

    int pixelWidth = width*tileSize;
    int pixelHeight = height*tileSize;

    size_t pixelsLen = pixelWidth*pixelHeight;
    std::unique_ptr<unsigned int> pixels((new unsigned int[pixelsLen]()));

    std::lock_guard<std::mutex> lock(mapCloudListMutex);

    paintRect(pixels.get(), pixelWidth,
        0, 0,
        0xFF000000,
        pixelWidth, pixelHeight
    );

    for (auto element = mapCloudList.begin(); element != mapCloudList.end(); element++) {
        MapCloud &mc = **element;

        int ix = mc.lat - centerLat + width/2 - 1;
        int iy = mc.lon - centerLon + height/2 - 1;

        if (ix < 0 || ix >= width || iy < 0 || iy >= height) break;

        paintRect(pixels.get(), pixelWidth,
            ix*tileSize, iy*tileSize,
            0xFF555555,
            tileSize, tileSize
        );

        for (int is = 0; is < subtileNum; is++) {
                
            bool free = mc.readerFree[is];
                
            char r = free ? 0 : 0xFF;
            char g = free ? 0xFF : 0;
            char b = 0;

            unsigned int color = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);

            color |= 0xFF000000;
            paintRect(pixels.get(), pixelWidth,
                ix*tileSize + border + is * subtileWidth, iy*tileSize + border,
                color,
                subtileWidth, tileSize - border*2
            );
        }
    }

    encodeImage(&img, pixels.get(), pixelWidth, pixelHeight);
}

static void getBlockFromCoords(Vec reference, Vec coords, int &bx, int &by, int &bz)
{
    /*
	bx = static_cast<int>(p.x - reference.x());
	by = static_cast<int>(p.z - reference.z());
	bz = static_cast<int>(reference.y() - p.y);
    */
    Vec diff = coords - reference;
    bx = diff.x();
    by = diff.z();
    bz = -diff.y();
}

static void getBlockFromCoords(Vec reference, Point p, int &bx, int &by, int &bz)
{
    Vec coords;
    coords << p.x, p.y, p.z;
    getBlockFromCoords(reference, coords, bx, by, bz);
}

static void getCoordsFromBlock(Vec reference, int bx, int by, int bz, Vec &coords)
{
    //p.x = bx + reference[0];
    //p.y = reference[1] - bz;
    //p.z = by + reference[2];
    coords << bx, bz, by;
    coords(1) = -coords(1);
    coords += reference;
}

static void getCoordsFromBlock(Vec reference, int bx, int by, int bz, Point &p)
{
    Vec coords;
    getCoordsFromBlock(reference, bx, by, bz, coords);
    p.x = coords.x();
    p.y = coords.y();
    p.z = coords.z();
}

static void extendColumn(int bx, int by, int bz, int sx, unsigned int* columns, int* minHeight = nullptr, int* maxHeight = nullptr)
{
	if (maxHeight && by > *maxHeight) *maxHeight = by;
	if (minHeight && by < *minHeight) *minHeight = by;
	int colindex = getColumnIndex(bx, bz, sx);
	int col = columns[colindex];
	if (by > col) columns[colindex] = by;
}

static void applyBlockToCloud(Vec origin, int bx, int by, int bz, Classification c, PointCloud *cloud) {
    Vec query_block_center;
    getCoordsFromBlock(origin, bx, by, bz, query_block_center);

    Vec block_center; block_center << 0.5, 0.5, 0.5;
    query_block_center += block_center;

    const pcln radius = M_SQRT2;
    std::vector<std::pair<size_t, pcln> > indices;
    RadiusResultSet<pcln, size_t> points(radius*radius, indices);
    cloud->findRadius(query_block_center.data(), points);

    for (size_t in = 0; in < points.size(); in++) {
        std::pair<size_t, pcln> pair = points.m_indices_dists[in];
        Point &p = cloud->getPoint(pair.first);
        p.classification = c;
    }
}

static Classification getBlockFromCloud(Vec origin, int bx, int by, int bz, PointCloud *cloud) {
    Vec query_block_center;
    getCoordsFromBlock(origin, bx, by, bz, query_block_center);

    Vec block_center; block_center << 0.5, 0.5, 0.5;
    query_block_center += block_center;

    const pcln radius = M_SQRT1_2;
    std::vector<std::pair<size_t, pcln> > indices;
    RadiusResultSet<pcln, size_t> points(radius*radius, indices);
    cloud->findRadius(query_block_center.data(), points);

    int classificationCounters[Classification::END] = { 0 };

    for (size_t in = 0; in < points.size(); in++) {
        std::pair<size_t, pcln> pair = points.m_indices_dists[in];
        Point &p = cloud->getPoint(pair.first);
        classificationCounters[p.classification]++;
    }

    int maxCount = 0;
    Classification maxClassification = Classification::END;
    for (int i = 0; i < Classification::END; i++) {
        int count = classificationCounters[i];
        if (count > maxCount) {
            maxCount = count;
            maxClassification = static_cast<Classification>(i);
        }
    }

    return maxClassification;
}


static std::mutex boxvisMutex;

// Position and size of the requested box (all block coordinates)
// Returns mutex locked box (unlock when done)
BoxResult& getBox(const long x, const long y, const long z, const long sx, const long sy, const long sz, const bool debug = false) {

    if (sx <= 0 || sy <= 0 || sz <= 0) return invalidBoxResult;

	// Power of 2 sizes
	int psx = (int)log2(sx);
	int psy = (int)log2(sy);
	int psz = (int)log2(sz);
	// Size mask per dimension
	int msx = sx - 1;
	int msy = sy - 1;
	int msz = sz - 1;

	// Box coordinates
	int bx = x >> psx;
	int by = y >> psy;
	int bz = z >> psz;

	assert(1 << psx == sx);
	assert(1 << psy == sy);
	assert(1 << psz == sz);

	//       //
	// Cache //
	//       //

	// Try cached box
	unsigned int boxHashCode;
	BoxResult &br = boxHash.at(bx, bz, boxHashCode);

	// Exclusive box access
	//std::lock_guard<std::mutex> brLock(br.mutex);
	br.mutex.lock();

	// Update access time
	long access = ++boxHashAccess;
	br.access = access;

	// Return cached if found
	if (!debug && br.valid &&
		br.x == x && br.y == y  && br.z == z &&
		br.sx == sx && br.sy == sy && br.sz == sz) {
		return br;
	}

    dtimer("box generation");

	//         //
	// Process //
	//         //

	// Not found in cache, update cached params
	br.valid = false;
	br.x = x;
	br.y = y;
	br.z = z;
	br.sx = sx;
	br.sy = sy;
	br.sz = sz;

    /*
	br.bx = bx;
	br.by = by;
	br.bz = bz;
    */

    //br.jsonOpen();
    br.exportOpen();

	// Precomputed strides
	int sxyz = sx*sy*sz;
	int sxz = sx*sz;

    /*
    br.blocks.resize(sxyz);
	br.columns.resize(sxz);
	br.groundCols.resize(sxz);
    std::vector<unsigned int> &blocks = br.blocks;
	std::vector<unsigned int> &columns = br.columns;
	std::vector<unsigned int> &groundCols = br.groundCols;
    */

    std::vector<unsigned int> blocks(sxyz, 0);
    std::vector<unsigned int> columns(sxz, 0);
    std::vector<unsigned int> groundCols(sxz, 0);

	//std::vector<unsigned int> blocks(sxyz);
	//std::vector<unsigned int> columns(sxz);

	int count = 0;
	int minHeight = sy;
	int maxHeight = -1;

	// x -> z
	// y -> x
	// z -> y

    static const int lat_min = MININT, lat_max = MAXINT, lon_min = MININT, lon_max = MAXINT;
	//static const int lat_min = 457, lat_max = 467, lon_min = 96, lon_max = 106;
	//static const int lat_min = 462, lat_max = 462, lon_min = 101, lon_max = 101;

    /*
	Vec &bounds_tl = br.bounds_tl;
	Vec &bounds_br = br.bounds_br;
	Vec &bounds_min = br.bounds_min;
	Vec &bounds_max = br.bounds_max;
    */

    Vec bounds_tl;
    Vec bounds_br;
    Vec bounds_min;
    Vec bounds_max;

	// Get bounds
	getCoordsFromBlock(origin, x, y, z, bounds_tl);
	getCoordsFromBlock(origin, x + sx, y + sy, z + sz, bounds_br);

	bounds_min = bounds_tl.cwiseMin(bounds_br);
	bounds_max = bounds_tl.cwiseMax(bounds_br);

	size_t num = 0;

	PointCloud all(50);
	PointCloud ground(20);

    MapCloud* cornerClouds[4];

	{
		//            //
		// Point load //
		//            //
		dtimer("point load");

		const int cornerNum = 4;
		const int corners[cornerNum][2] = {
			{ bounds_min.x(), bounds_min.y() }, { bounds_max.x(), bounds_min.y() },
			{ bounds_min.x(), bounds_max.y() }, { bounds_max.x(), bounds_max.y() }
		};

		int latlonCount = 0;
		int latlon[cornerNum][3];

		for (int i = 0; i < cornerNum; i++) {

			cornerClouds[i] = nullptr;

			int lat = corners[i][0] / 1000;
			int lon = corners[i][1] / 1000;

			lat = std::max(std::min(lat, lat_max), lat_min);
			lon = std::max(std::min(lon, lon_max), lon_min);

			bool duplicate = false;
			for (int j = 0; j < latlonCount; j++) {
				if (latlon[j][1] == lat && latlon[j][2] == lon) {
					duplicate = true;
					break;
				}
			}
			if (duplicate) continue;

			latlon[latlonCount][0] = i;
			latlon[latlonCount][1] = lat;
			latlon[latlonCount][2] = lon;
			latlonCount++;
		}

		for (int i = 0; i < latlonCount; i++) {
			int index = latlon[i][0];
			int lat = latlon[i][1];
			int lon = latlon[i][2];
			MapCloud *mc = getMapCloud(lat, lon);
			cornerClouds[index] = mc;
			mc->load(&all, &ground, bounds_min.x(), bounds_min.y(), bounds_max.x(), bounds_max.y());
		}

	}

	{
		//              //
		// Quantization //
		//              //
		dtimer("quantization");
		size_t num = all.getPointNum();
		pointsLoaded += num;
		for (size_t i = 0; i < num; i++) {
			Point &p = all.getPoint(i);

            int bx, by, bz;
			getBlockFromCoords(bounds_tl, p, bx, by, bz);

            /*
			if (by < 0 || by > sy) continue;
            */

            if (bx < 0 || bx >= sx ||
                by < 0 || by >= sy ||
                bz < 0 || bz >= sz) continue;

            //vassert(sx > 0 && sxz > 0, "Invalid size: %d %d", sx, sxz);

            /*vassert(
                bx >= 0 && bx < sx &&
                by >= 0 && by < sy &&
                bz >= 0 && bz < sz, "Block coords out of bounds: %d %d %d", bx, by, bz);
            */

			int index = getBlockIndex(bx, by, bz, sx, sxz);

            /*vassert(
                bx >= 0 && bx < sx &&
                by >= 0 && by < sy &&
                bz >= 0 && bz < sz, "Block coords out of bounds: %d %d %d", bx, by, bz);
            */
            //vassert(index >= 0 && index < blocks.size(), "Index out of bounds: %d  bx %d by %d bz %d sx %d sy %d sz %d", index, bx, by, bz, sx, sy, sz);

			blocks[index] = p.classification;

            /*
			if (by < minHeight) minHeight = by;
            if (by > maxHeight) maxHeight = by;
            */

            //br.exportWrite(bx + 0.5, bz + 0.5, by + 0.5, -10);

			count++;
		}
	}

	num = count;


    /*
    for (size_t i = 0; i < all.getPointNum(); i++) {
        Point &p = all.getPoint(i);

        //br.exportWrite(p.x - bounds_tl.x(), p.y - bounds_tl.y(), p.z - bounds_tl.z(), -100);

        //br.jsonWrite(p.x - bounds_tl.x(), p.y - bounds_tl.y(), p.z - bounds_tl.z(), -100);
        //br.exportWrite(p.x - bounds_min.x(), p.y - bounds_min.y(), p.z - bounds_min.z(), -100);
    }

    for (size_t i = 0; i < ground.getPointNum(); i++) {
        Point &p = ground.getPoint(i);

        //br.exportWrite(p.x - bounds_min.x(), p.y - bounds_min.y(), p.z - bounds_min.z(), -101);
        //br.jsonWrite(p.x - bounds_tl.x(), p.z - bounds_tl.z(), p.y - bounds_tl.y(), -101);
    }
    */

	unsigned int *cblocks = blocks.data();
	
    /*
    // Debug / testing
    for (int i = 0; i < sxyz; i++) {
        unsigned int &cv = cblocks[i];
        int bx, by, bz;
        getIndexBlock(i, bx, by, bz, sx, sz);

        if (by == 5) {


            Classification lidar = NONE;
            for (int iy = 0; iy < sy; iy++) {
                int ind = getBlockIndex(bx, iy, bz, sx, sxz);
                Classification lid = static_cast<Classification>(cblocks[ind]);
                if (lid != 0) lidar = lid;
            }

            Point cqp;
            getCoordsFromBlock(bounds_min.data(), bx, by, bz, cqp);
            getBlockFromCoords(origin.data(), cqp, bx, by, bz);
            bz = 1000 - bz;
            getCoordsFromBlock(origin.data(), bx, by, bz, cqp);

            ClassificationQuery cq;
            cq.x = cqp.x;
            cq.y = cqp.y;
            cq.lidar = lidar;
            cq.flags = ClassificationFlags::DEFAULT;
            cv = mapCloud->getSpecializedClassification(cq);
        }
        else {
            cv = Classification::NONE;
        }

    }
    //*/

	// Process points if any found
    if (num > 0) {

        /*
        {
            //                                      //
            // Count classifications for each block //
            //                                      //
            dtimer("count");
            Vec block_center; block_center << 0.5, 0.5, 0.5;
            for (int i = 0; i < sxyz; i++) {
                unsigned int &cv = cblocks[i];

                if (cv != 0) {
                    int bx = i & msx;
                    int bz = (i >> psx) & msz;
                    int by = (i >> psx) >> psz;

                    Classification c = getBlockFromCloud(bounds_tl, bx, by, bz, &all);
                    if (c != Classification::END) cv = c;
                }

                //c |= (i%0xF) << 16;
            }
        }
        //*/

        //*
        {
            //                    //
            // Initialize columns //
            //                    //
            dtimer("columns");
            for (int i = 0; i < sxyz; i++) {
                unsigned int &cv = cblocks[i];

                if (cv != 0) {
                    int bx = i & msx;
                    int bz = (i >> psx) & msz;
                    int by = (i >> psx) >> psz;

                    extendColumn(bx, by, bz, sx, columns.data(), &minHeight, &maxHeight);
                    if (cv == Classification::GROUND) extendColumn(bx, by, bz, sx, groundCols.data());
                }
            }
        }
        //*/

        //*
        {
            //               //
            // Building fill //
            //               //
            dtimer("building fill");
            //dtimerInit(slanted_roof_search, "slanted roof search");

            for (int i = 0; i < sxyz; i++) {
                unsigned int &cv = cblocks[i];
                if (cv == Classification::BUILDING) {

                    int bx, by, bz;

                    getIndexBlock(i, bx, by, bz, sx, sz);

                    //if (by > 130) c = Classification::WATER;
                    int index = i - sxz;
                    int iy = by - 1;
                    while (iy > minHeight) {
                        cblocks[index] = Classification::BUILDING;
                        index -= sxz;
                        iy--;
                    }

                    extendColumn(bx, minHeight, bz, sx, groundCols.data());
                }
            }
        }

        {
            //                       //
            // Spatial tree (ground) //
            //                       //
            dtimer("kdtree ground");
            ground.build();
        }


        //*
        {
            //             //
            // Ground fill //
            //             //
            dtimer("ground fill");
            dtimerInit(ground_search, "ground search");

            Vec block_center; block_center << 0.5, 0.5, 0.5;

            for (int iz = 0; iz < sz; iz++) {
                for (int ix = 0; ix < sx; ix++) {

                    int colindex = getColumnIndex(ix, iz, sx);
                    int col = groundCols[colindex];
                    if (col > 0) continue;

                    int bx = ix, bz = iz;
                    int by = 0;
                    bool found = false;

                    {
                        dtimerStart(ground_search);

                        const double min_diff = 0.5;

                        const int max_iters = 10;
                        Vec query_block_center;
                        size_t ret_index;
                        pcln out_dist_sqr;
                        pcln diff = min_diff + 1;
                        Point *point;
                        getCoordsFromBlock(bounds_tl, bx, by, bz, query_block_center);
                        query_block_center += block_center;

                        int i = 0;
                        //for (i = 0; i < num_iters; i++) {
                        for (i = 0; i < max_iters && diff > min_diff; i++) {
                            found = ground.findNearest(query_block_center.data(), ret_index, out_dist_sqr);

                            if (!found) break;

                            point = &ground.getPoint(ret_index);
                            diff = abs(query_block_center.z() - point->z);
                            query_block_center.z() = point->z;

                            //br.exportWrite(query_block_center.x() - bounds_tl.x(), query_block_center.z() - bounds_tl.z(), query_block_center.y() - bounds_tl.y(), -50 - i);
                            br.exportWrite(query_block_center.x() - bounds_tl.x(), i * 5, query_block_center.y() - bounds_tl.y(), -100 - i);
                        }

                        if (i > 0) {
                            getBlockFromCoords(bounds_tl, *point, bx, by, bz);
                            bx = ix;
                            bz = iz;
                            //br.exportWrite(bx + 0.5, by + 0.5, bz + 0.5, -50);
                            found = true;
                        }

                        if (found) {
                            Point p;
                            p.x = query_block_center.x();
                            p.y = query_block_center.y();
                            p.z = point->z;
                            all.addPoint(p);
                        }

                        dtimerStop(ground_search);
                    }

                    by = by >= 0 ? by < sy ? by : sy - 1 : 0;

                    if (by < 0) by = 0;
                    if (by >= sy) by = sy - 1;

                    //br.jsonWrite(bx + 0.5, by + 0.5, bz + 0.5, -13);

                    groundCols[colindex] = by;

                }
            }

            {
                dtimer("ground set");
                for (int iy = 0; iy < sy; iy++) {
                    for (int iz = 0; iz < sz; iz++) {
                        for (int ix = 0; ix < sx; ix++) {
                            int index = getBlockIndex(ix, iy, iz, sx, sxz);
                            int colindex = getColumnIndex(ix, iz, sx);
                            int col = groundCols[colindex];
                            if (iy <= col) {
                                cblocks[index] = Classification::GROUND;
                                extendColumn(ix, iy, iz, sx, columns.data(), &minHeight, &maxHeight);
                            }
                        }
                    }
                }
            }

        }
        //*/

        {
            //                    //
            // Spatial tree (all) //
            //                    //
            dtimer("kdtree all");
            all.build();
        }

        /*
        {
            //                     //
            // Slanted roof search //
            //                     //
            dtimer("slanted roof search");
            for (int iz = 0; iz < sz; iz++) {
                for (int ix = 0; ix < sx; ix++) {
                    int bx = ix;
                    int bz = iz;
                    int colindex = getColumnIndex(bx, bz, sx);
                    int by = columns[colindex];
                    int index = getBlockIndex(bx, by, bz, sx, sxz);
                    unsigned int &cv = cblocks[index];

                    if (cv == Classification::BUILDING)
                    ///
                    {
                        //dtimerStart(slanted_roof_search);
                        // Slanted roof search
                        Vec query_block_center;
                        getCoordsFromBlock(bounds_tl, bx, by, bz, query_block_center);

                        query_block_center += Vec::Constant(0.5);

                        const pcln radius = 2;
                        std::vector<std::pair<size_t, pcln> > indices;
                        RadiusResultSet<pcln, size_t> points(radius*radius, indices);


                        all.findRadius(query_block_center.data(), points);

                        double slantAngle = 45;
                        //double acceptedAngleDelta = 15;
                        double dotThreshold = 0.40;

                        size_t buildingPoints = 0;
                        size_t slantedPoints = 0;
                        //double avgAngle = 0;
                        double avgDot = 0;
                        for (size_t in = 0; in < points.size(); in++) {
                            std::pair<size_t, pcln> pair = points.m_indices_dists[in];
                            Point &p = all.getPoint(pair.first);
                            if (p.classification != Classification::BUILDING) continue;
                            buildingPoints++;
                            Vec buildingPoint(p.x, p.y, p.z);
                            Vec deltaVector = buildingPoint - query_block_center;
                            deltaVector.normalize();
                            double dot = deltaVector.dot(Eigen::Vector3d::UnitZ());
                            avgDot += abs(dot);
                            //double angle = acos(dot) * 180 / M_PI;
                            //avgAngle += abs(angle);
                            //if (abs(angle - slantAngle) < acceptedAngleDelta) slantedPoints++;
                        }
                        //avgAngle /= buildingPoints;
                        avgDot /= buildingPoints;
                        if (avgDot > dotThreshold) cv = Classification::BUILDING_ROOF_TILED_ORANGE;

                        //dtimerStop(slanted_roof_search);
                    }
                    ///
                }
            }
        }
            //*/

            
        /*
        std::vector<unsigned int> buildCols(sxz, 0);

        for (int iz = 0; iz < sz; iz++) {
            for (int ix = 0; ix < sx; ix++) {
                int bx = ix;
                int bz = iz;
                int colindex = getColumnIndex(bx, bz, sx);
                int by = columns[colindex];
                int index = getBlockIndex(bx, by, bz, sx, sxz);
                unsigned int &cv = cblocks[index];
                if (cv == Classification::BUILDING) extendColumn(bx, by, bz, sx, buildCols.data());
            }
        }

        for (int i = 0; i < sxyz; i++) {
            unsigned int &cv = cblocks[i];
            int bx, by, bz;
            getIndexBlock(i, bx, by, bz, sx, sz);
            int colindex = getColumnIndex(bx, bz, sx);
            if (by > minHeight && by < buildCols[colindex]) cv = Classification::BUILDING;
        }
        */

        //*/


		//*
		{
			//                //
			// Specialization //
			//                //
			dtimer("specialization");
			for (int iz = 0; iz < sz; iz++) {
				for (int ix = 0; ix < sx; ix++) {

					int bx = ix;
					int bz = iz;
					int colindex = getColumnIndex(bx, bz, sx);
					int by = columns[colindex];
					int index = getBlockIndex(bx, by, bz, sx, sxz);
					unsigned int &cv = cblocks[index];
					int c = cv & 0xFF;

					ClassificationQuery cq;
					Point cq_point;
					getCoordsFromBlock(bounds_tl, bx, by, bz, cq_point);
					cq.x = cq_point.x;
					cq.y = cq_point.y;
					cq.corners = cornerClouds;
					cq.lidar = static_cast<Classification>(c);
					MapCloud *mapCloud;
					int mx;
					int my;
					Classification sc = MapCloud::getSpecializedClassification(cq, &mapCloud, &mx, &my);
                    if (sc == Classification::NONE) continue;

                    cv = sc;

                    //br.jsonWrite(bx, by, bz, -1);


                    // Apply specialization to point cloud
                    applyBlockToCloud(bounds_tl, bx, by, bz, (Classification)cv, &all);
                    applyBlockToCloud(bounds_tl, bx, by, bz, (Classification)cv, &ground);

				}
			}
		}
		//*/
        
		//*
		{
			//           //
			// Filtering //
			//           //
			dtimer("filtering");

            Vec block_center; block_center << 0.5, 0.5, 0.5;
            Vec query_block_center;

            for (auto &filter : classificationFilters) {
                dtimer(filter.name.c_str());
                for (int iz = 0; iz < sz; iz++) {
				    for (int ix = 0; ix < sx; ix++) {

					    int bx = ix;
					    int bz = iz;
					    int colindex = getColumnIndex(bx, bz, sx);
					    int by = columns[colindex];
					    int index = getBlockIndex(bx, by, bz, sx, sxz);
					    unsigned int &cv = cblocks[index];
					    int c = cv & 0xFF;

                    //br.jsonWrite(bx, by, bz, c);

                        if (std::find(filter.sources.begin(), filter.sources.end(), c) == filter.sources.end()) continue;

                        getCoordsFromBlock(bounds_tl, bx, by, bz, query_block_center);
                        query_block_center += block_center;

                        //br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() - bounds_min.z(), -3);

                        const Classification target = filter.target;
                        const pcln radius = filter.radius;
                        const pcln thresholdRatio = filter.thresholdRatio;

                        std::vector<std::pair<size_t, pcln> > indices;
                        RadiusResultSet<pcln, size_t> points(radius*radius, indices);
                        all.findRadius(query_block_center.data(), points);

                        int sourcePoints = 0;
                        int targetPoints = 0;

                        bool targetClosest = target == Classification::NONE;
                        pcln minDist = INFINITY;
                        int minIndex = -1;

                        //br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() - bounds_min.z(), -3);

                        for (size_t in = 0; in < points.size(); in++) {
                            std::pair<size_t, pcln> pair = points.m_indices_dists[in];
                            Point &p = all.getPoint(pair.first);

                            if (p.classification != Classification::NONE && p.classification != Classification::UNASSIGNED) {
                                bool isTarget = targetClosest || p.classification == target;
                                bool isSource = p.classification == c;
                                if (isTarget) targetPoints++;
                                if (isSource) sourcePoints++;
                                if (targetClosest && !isSource && pair.second < minDist) {
                                    minDist = pair.second;
                                    minIndex = pair.first;
                                }
                            }
                        }

                        
                        pcln targetTotalRatio = points.size() > 0 ? ((pcln)targetPoints / points.size()) : 0;
                        //pcln targetSourceRatio = sourcePoints > 0 ? targetPoints / sourcePoints : targetPoints;
                        pcln diffRatio = points.size() > 0 ? ((pcln)targetPoints - sourcePoints) / points.size() : 0;

                        //br.jsonWrite(bx, by + ratio * 30, bz, -4);
                        //br.jsonWrite(bx, by + 30, bz, -5);

                        //br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() + ratio * 30 - bounds_min.z(), -4);

                        //br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() + 30 - bounds_min.z(), -7);
                        //br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() + diffRatio*30 - bounds_min.z(), diffRatio > 0 ? -4 : -5);

                        //br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() + sourcePoints*0.25 - bounds_min.z(), -4);
                        //br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() + targetPoints*0.25 - bounds_min.z(), -5);
                        //br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() + points.size()*0.25 - bounds_min.z(), -6);

                        bool changed = false;
                        if (diffRatio > thresholdRatio) {
                            if (targetClosest) {
                                if (minIndex != -1) {
                                    cv = all.getPoint(minIndex).classification;
                                    changed = true;
                                }
                            } else {
                                cv = target;
                                changed = true;
                            }
                            //br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() + 40 - bounds_min.z(), -5);
                        }

                        if (changed) applyBlockToCloud(bounds_tl, bx, by, bz, (Classification)cv, &all);

                    }

				}
			}
		}
		//*/

        // -1 means no water found
        int waterSurfaceLevel = -1;

        //*
        {
            //                    //
            // Water equalization //
            //                    //
            dtimer("water equalization");

            Vec block_center; block_center << 0.5, 0.5, 0.5;
            Vec query_block_center;


            int minHeight = std::numeric_limits<int>::max();
            std::vector<int> waterHeights;
            waterHeights.reserve(sx*sz);

            // Gather all water block heights in the box
            for (int iz = 0; iz < sz; iz++) {
                for (int ix = 0; ix < sx; ix++) {
                    int bx = ix;
                    int bz = iz;
                    int colindex = getColumnIndex(bx, bz, sx);
                    int by = columns[colindex];
                    int index = getBlockIndex(bx, by, bz, sx, sxz);
                    unsigned int &cv = cblocks[index];
                    int c = cv & 0xFF;

                    if (c == Classification::WATER) {
                        waterHeights.push_back(by);
                        if (by < minHeight) minHeight = by;
                    }
                }
            }

            int surfy = -1;
            if (waterHeights.size() > 0) {
                // Get the median height of a water block
                std::nth_element(waterHeights.begin(), waterHeights.begin() + waterHeights.size() / 2, waterHeights.end());
                int medHeight = waterHeights[waterHeights.size() / 2];
                surfy = medHeight;
            }

            if (surfy != -1) {
                // Safety measures :)
                surfy = surfy >= 0 ? surfy < sy ? surfy : surfy - 1 : 0;

                waterSurfaceLevel = surfy;

                // Adjust all water blocks towards the water level
                for (int iz = 0; iz < sz; iz++) {
                    for (int ix = 0; ix < sx; ix++) {

                        int bx = ix;
                        int bz = iz;
                        int colindex = getColumnIndex(bx, bz, sx);
                        unsigned int &by = columns[colindex];
                        int index = getBlockIndex(bx, by, bz, sx, sxz);
                        unsigned int &cv = cblocks[index];
                        int c = cv & 0xFF;

                        if (c != Classification::WATER) {
                            continue;
                        }

                        //if (by != ay) {
                        int step = by < surfy ? 1 : -1;
                        for (int iy = by; iy != surfy; iy += step) {
                            cblocks[getBlockIndex(bx, iy, bz, sx, sxz)] = Classification::NONE;
                            //applyBlockToCloud(bounds_tl, ax, iy, az, Classification::NONE, &all);
                        }
                        cblocks[getBlockIndex(bx, surfy, bz, sx, sxz)] = Classification::WATER;

                        // Fix column height
                        by = surfy;

                    }
                }
            }
        }
        //*/

        
        //*
        {
            //                                         //
            // Water deepening (requires equalization) //
            //                                         //
            dtimer("water deepening");

            // If there is any water to deepen
            if (waterSurfaceLevel != -1) {

                int by = waterSurfaceLevel;

                for (int iz = 0; iz < sz; iz++) {
                    for (int ix = 0; ix < sx; ix++) {

                        int bx = ix;
                        int bz = iz;
                        int index = getBlockIndex(bx, by, bz, sx, sxz);
                        unsigned int &cv = cblocks[index];
                        int c = cv & 0xFF;

                        if (c != Classification::WATER) continue;

                        const int directions = 8;
                        int dirvec[directions][2] = {
                            { -1, -1 }, {  0, -1 }, {  1, -1 },
                            { -1,  0 }, {  1,  0 },
                            { -1,  1 }, {  0,  1 }, {  1,  1 }
                        };

                        int maxDist = 20;

                        int shoreDist;
                        for (shoreDist = 0; shoreDist < maxDist; shoreDist++) {
                            int* offset = dirvec[(shoreDist * 2) % directions];
                            int shx = bx + offset[0] * shoreDist;
                            int shz = bz + offset[1] * shoreDist;
                            if (shx < 0 || shx >= sx || shz < 0 || shz >= sz) continue;
                            unsigned int shore = cblocks[getBlockIndex(shx, by, shz, sx, sxz)] & 0xFF;
                            if (shore != Classification::WATER) {
                                break;
                            }
                        }

                        double depth = shoreDist;

                        //double depth = 1 / (1 + exp(-(shoreDist - maxDist*0.5)*0.3)) * 20;

                        for (int iy = 1; iy < depth; iy++) {
                            int bdy = by - iy;
                            if (bdy < 0) break;
                            cblocks[getBlockIndex(bx, bdy, bz, sx, sxz)] = Classification::WATER;
                        }

                    }
                }
            }

        }
        //*/




        {
            //                                          //
            // Classification + custom blocks -> blocks //
            //                                          //
            dtimer("transform");

            // TODO: only use uint8 for cblocks, but then convert into the uint32
            // shape used below
            for (int i = 0; i < sxyz; i++) {
                unsigned int &cv = cblocks[i];

                int bx = i & msx;
                int bz = (i >> psx) & msz;
                int by = (i >> psx) >> psz;

                if (cv && !debug) cv = classificationToBlock(cv);

                if (cv > 0) {
                    //br.jsonWrite(bx + 0.5, by + 0.5, bz + 0.5, -10);
                    extendColumn(bx, by, bz, sx, columns.data(), &minHeight, &maxHeight);
                }

                //c |= (i%0xF) << 16;
            }
        }

        /*
        {
            //                             //
            // Water deepening (via cloud) //
            //                             //
            dtimer("water deepening");

            Vec block_center; block_center << 0.5, 0.5, 0.5;
            Vec query_block_center;

            for (int iz = 0; iz < sz; iz++) {
                for (int ix = 0; ix < sx; ix++) {

                    int bx = ix;
                    int bz = iz;
                    int colindex = getColumnIndex(bx, bz, sx);
                    int by = columns[colindex];
                    int index = getBlockIndex(bx, by, bz, sx, sxz);
                    unsigned int &cv = cblocks[index];
                    int c = cv & 0xFF;

                    if (c != Classification::WATER) continue;

                    getCoordsFromBlock(bounds_tl, bx, by, bz, query_block_center);
                    query_block_center += block_center;

                    const pcln radius = 10;

                    std::vector<std::pair<size_t, pcln> > indices;
                    RadiusResultSet<pcln, size_t> points(radius*radius, indices);
                    all.findRadius(query_block_center.data(), points);

                    pcln vertPenalty = 0;
                    pcln waterScore = 0;
                    pcln totalScore = 0;

                    for (size_t in = 0; in < points.size(); in++) {
                        std::pair<size_t, pcln> pair = points.m_indices_dists[in];
                        Point &p = all.getPoint(pair.first);
                        //br.jsonWrite(p.x - bounds_min.x(), p.z - bounds_min.z(), p.y - bounds_min.y(), -21);
                        pcln score = 1 - vertPenalty*(query_block_center.z() - p.z);
                        if (p.classification == Classification::WATER) {
                            waterScore += score;
                        } else {
                            totalScore += score;
                        }
                    }

                    //br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.z() + waterScore - bounds_min.z(), query_block_center.y() - bounds_min.y(), -22);
                    //br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.z() + totalScore - bounds_min.z(), query_block_center.y() - bounds_min.y(), -23);

                    pcln ratio = totalScore > 0 ? waterScore / totalScore : 0;

                    double depth = ratio * 5;

                    for (int iy = 1; iy < depth; iy++) {
                        int bdy = by - iy;
                        if (bdy < 0) break;
                        cblocks[getBlockIndex(bx, bdy, bz, sx, sxz)] = Classification::WATER;
                    }

                }
            }
        }
        //*/

        
		/*
		{
			//                 //
			// Water deepening old //
			//                 //
			dtimer("water");
			for (int iz = 0; iz < sz; iz++) {
				for (int ix = 0; ix < sx; ix++) {

					int bx = ix;
					int bz = iz;
					int colindex = getColumnIndex(bx, bz, sx);
					int by = columns[colindex];
					int index = getBlockIndex(bx, by, bz, sx, sxz);
					unsigned int &cv = cblocks[index];
					int c = cv & 0xFF;

					Point p;
					int mx, my;
					getCoordsFromBlock(bounds_tl, bx, by, bz, p);
					MapCloud* mapCloud = MapCloud::fromCorners(br.cornerClouds, p.x, p.y, &mx, &my);

					switch (c)
					{
					case Classification::WATER:
						// Add depth

						const int directions = 8;
						int dirvec[directions][2] = {
							{ -1, -1 }, { 0, -1 }, { 1, -1 },
							{ -1, 0 }, { 1, 0 },
							{ -1, 1 }, { 0, 1 }, { 1, 1 }
						};

						ClassificationQuery cq;
						cq.lidar = Classification::WATER;

						int maxDist = 20;

						int shoreDist;
						for (shoreDist = 0; shoreDist < maxDist; shoreDist++) {
							int* offset = dirvec[shoreDist * 3];
							Classification shore = mapCloud->getMapPointClassification(cq,
								mx + offset[0] * shoreDist,
								my + offset[1] * shoreDist
							);
							if (shore != Classification::WATER) {
								break;
							}
						}

						double depth = 1 / (1 + exp(-(shoreDist - maxDist*0.5)*0.3)) * 20;

						for (int iy = 1; iy < depth; iy++) {
							int bdy = by - iy;
							if (bdy < 0) break;
							cblocks[getBlockIndex(bx, bdy, bz, sx, sxz)] = Classification::WATER;
						}

						break;
					}
				}
			}
		}
		//*/


		/*


        //
        //
        //
        continue;
        ///
        ///
        ///
        switch (c)
        {
        case Classification::WATER:
        case Classification::VEGETATION_LOW:
        case Classification::VEGETATION_MEDIUM:
        case Classification::GROUND_ASPHALT:


        Vec query_block_center;
        getCoordsFromBlock(bounds_tl, bx, by, bz, query_block_center);
        Vec block_center; block_center << 0.5, 0.5, 0.5; query_block_center += block_center;

        const pcln depthRadius = 20;
        const pcln filterRadius = 6;
        const pcln maxDepth = 20;
        const pcln minWaterRatio = 0.2;
        const pcln maxWaterRatio = 0.45;

        const pcln radius = depthRadius;
        std::vector<std::pair<size_t, pcln> > indices;
        RadiusResultSet<pcln, size_t> points(radius*radius, indices);
        all.findRadius(query_block_center.data(), points);

        int depthWaterPoints = 0;
        int filterWaterPoints = 0;
        int filterTotalPoints = 0;
        pcln minDist = INFINITY;
        int minIndex = -1;

        br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() - bounds_min.z(), -3);

        for (size_t in = 0; in < points.size(); in++) {
        std::pair<size_t, pcln> pair = points.m_indices_dists[in];
        Point &p = all.getPoint(pair.first);

        bool water = p.classification == Classification::WATER;
        if (water) {
        depthWaterPoints++;
        } else {
        if (pair.second < minDist) {
        minDist = pair.second;
        minIndex = pair.first;
        }
        }

        if (pair.second < filterRadius) {
        if (water) filterWaterPoints++;
        filterTotalPoints++;
        }
        }

        double filterWaterRatio = filterTotalPoints > 0 ? (double)filterWaterPoints / filterTotalPoints : 0;
        if (c == Classification::WATER && filterWaterRatio < minWaterRatio && minIndex > -1) {
        cv = all.getPoint(minIndex).classification;
        br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() + 35 - bounds_min.z(), -7);
        } else if (c != Classification::WATER) {
        if (filterWaterRatio > maxWaterRatio) {
        cv = Classification::WATER;
        br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() + 40 - bounds_min.z(), -8);
        } else {
        break;
        }
        }

        double depthWaterRatio = (double)depthWaterPoints / points.size();

        double depth = depthWaterRatio*maxDepth; //(1 - 1 / (-0.5 + exp(2 * depthWaterRatio))) * maxDepth;

        for (int iy = 1; iy < depth; iy++) {
        int bdy = by - iy;
        if (bdy < 0) break;
        cblocks[getBlockIndex(bx, bdy, bz, sx, sxz)] = Classification::WATER;
        br.jsonWrite(bx, bdy, bz, -7);
        }

        br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() + filterWaterRatio * 30 - bounds_min.z(), -5);
        //br.jsonWrite(query_block_center.x() - bounds_min.x(), query_block_center.y() - bounds_min.y(), query_block_center.z() + depthWaterRatio * 30 - bounds_min.z(), -6);

        break;

        }


		{
			//           //
			// Filtering - Old //
			//           //
			dtimer("filtering");
			for (int iz = 0; iz < sz; iz++) {
				for (int ix = 0; ix < sx; ix++) {

					int bx = ix;
					int bz = iz;
					int colindex = getColumnIndex(bx, bz, sx);
					int by = columns[colindex];
					int index = getBlockIndex(bx, by, bz, sx, sxz);
					unsigned int &cv = cblocks[index];
					int c = cv & 0xFF;

					switch (c)
					{
					case Classification::VEGETATION_LOW:

						// Unsupported top edge case
						if (by >= sy - 1) break;

						// Spiral search for water
						const int spiralEdges = 4;

						int ox = 0;
						int oz = 0;
						int edge = 0;

						for (int i = 0; i < spiralEdges; i++) {
							if (i % 2 == 0) edge++;
							for (int j = 0; j < edge; j++) {
								switch (i % 4) {
									case 0: ox++; break;
									case 1: oz++; break;
									case 2: ox--; break;
									case 3: oz--; break;
								}

								int bxo = bx + ox;
								int bzo = bz + oz;

								int spIndexAbove = getBlockIndex(bxo, by + 1, bzo, sx, sxz);
								unsigned int &spva = cblocks[spIndexAbove];
								int spca = spva & 0xFF;

								// Stop the search if hit shore
								if (spca != Classification::NONE) break;

								int spIndex = getBlockIndex(bxo, by, bzo, sx, sxz);
								unsigned int &spv = cblocks[spIndex];
								int spc = spv & 0xFF;

								if (spc == Classification::WATER) {
									// Found water, so this is probably non-water noise
									cv = Classification::WATER;
									break;
								}

								//plog("%d %d %d %d", i, j, ox, oy);
							}
						}

						break;
					}

				}
			}
		}
		//*/

    }

    //br.jsonClose();
    br.exportClose();

	// Fix height so it's above the highest block
	maxHeight++;

	// For an empty box, report as such
	if (count == 0) maxHeight = -2;

	{
		dtimer("serialization");

		amf::Serializer serializer;
		serializer << amf::AmfVector<unsigned int>(blocks);
		serializer << amf::AmfVector<unsigned int>(columns);

		amf::v8 arrays = serializer.data();

		if (!br.data) br.data = new amf::v8();
		amf::v8 *data = br.data;

		const int size_int = 4;
		size_t dataSize = 3 * size_int + arrays.size() + 1 * size_int;
		data->resize(dataSize);
		vassert(data->size() == dataSize, "%d %d", (int)data->size(), dataSize);

		amf::u8 *p = data->data();
		writeInt(p, bx); p += size_int;
		writeInt(p, by); p += size_int;
		writeInt(p, bz); p += size_int;
		memcpy(p, arrays.data(), arrays.size()); p += arrays.size();
		writeInt(p, maxHeight); p += size_int;

		br.valid = true;
	}

	++boxesCreated;

	return br;
}


void GKOTHandleBox(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{

    long x = 0;
    long y = 0;
    long z = 0;
    long sx = 1;
    long sy = 512;
    long sz = 1;
    bool debug = false;

    plogScope();

    const char *qs = info->query_string;
    size_t ql = strlen(info->query_string);

    x  = getParamLong(qs, ql, "x");
    y  = getParamLong(qs, ql, "y");
    z  = getParamLong(qs, ql, "z");
    sx = getParamLong(qs, ql, "sx");
    sy = getParamLong(qs, ql, "sy");
    sz = getParamLong(qs, ql, "sz");
    debug = getParamBool(qs, ql, "debug");

    debugPrint("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
    debugPrint("<html><body>");
    debugPrint("Hello!<br><pre>");

    debugPrint("x: %d\n", x);
    debugPrint("y: %d\n", y);
    debugPrint("z: %d\n", z);
    debugPrint("sx / w: %d\n", sx);
    debugPrint("sy    : %d\n", sy);
    debugPrint("sz / h: %d\n", sz);

    //debugPrint("%d points found\n", num);
    //debugPrint("%d points inside, %d points outside\n", count, num-count);

    //debug("%d points", num);
    //debugPrint("%d points\n", count);

    //plog("box %ld %ld %ld %ld %ld %ld", x, y, z, sx, sy, sz);

	BoxResult &br = getBox(x, y, z, sx, sy, sz, debug);

    if (&br == &invalidBoxResult) {
        printf("Invalid box %ld %ld %ld %ld %ld %ld\n", x, y, z, sx, sy, sz);
        mg_send_http_error(conn, 400, "Invalid box");
        return;
    }

	{
		dtimer("send");
		br.send(conn);
		br.mutex.unlock();
		++boxesSent;
	}

	debugPrint("</pre></body></html>\n");

    //serializer << amfblocks;
    //serializer << amfcolumns;





    //data.clear();
    //data.reserve(3 * 4 + arrays.size() + 1 * 4);


    
    //printf(" br %d %d %d ", br.x, br.z, br.valid);

    //printf(" bef size() %d, dataSize %d ", data.size(), dataSize);


    //printf(" aft size() %d, dataSize %d ", data.size(), dataSize);

    //assert(data.size() == dataSize);




    //pushInt(data, by);
    //pushInt(data, bz);

    //pushInt(data, bx);
    //pushInt(data, by);
    //pushInt(data, bz);

    //printf("%d %d  %d  %d, %d", data.capacity(), data.size(), arrays.size(), data.end() - data.begin(), arrays.end() - arrays.begin());
    
    //data.insert(data.end(), arrays.begin(), arrays.end());
    //pushInt(data, maxHeight);


	/*
    debugPrint("%d max height\n", maxHeight);
    debugPrint("bx %d  bz %d  by %d\n", bx, bz, by);

    if (debug) {
        //printArray(conn, static_cast<unsigned char*>(data.data()), data.size());
        unsigned int *pixels;
        int width;
        int height;

        createColumnVis(columns.data(), sx, sz, &pixels, width, height);
        printImage(conn, pixels, width, height);
        free(pixels);

        createBlockVis(cblocks, sx, sy, sz, &pixels, width, height);
        printImage(conn, pixels, width, height);
        free(pixels);
    } else {
        dtimer("send");
        br.send(conn);
        boxesSent++;
    }
	*/

    //mapCloudHash.mutex.unlock();
    //br.mutex.unlock();
    //return status;
    //return "query";

    //printf("  GKOT %d, %d, %d   %d bytes\n", bx, by, bz, data.size());
}

enum TileType {
	TYPE_INVALID,
	TYPE_LIDAR,
	TYPE_MAP
};

void GKOTHandleTile(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
	std::string request = info->request_uri;

	std::vector<std::string> ext_spl = split(request, '.');

	if (ext_spl.size() != 2) { mg_send_http_error(conn, 400, "Too many periods!"); return; }
	if (ext_spl[1] != "png") { mg_send_http_error(conn, 400, "Unsupported file extension!"); return; }

	std::vector<std::string> request_tokens = split(request, '_');

	if (request_tokens.size() != 8) { mg_send_http_error(conn, 400, "Invalid number of tokens!"); return; }

	int token_index = 1;

	std::string token_type = request_tokens[token_index++];

	TileType type =
		token_type == "lidar" ? TYPE_LIDAR :
		token_type == "map" ? TYPE_MAP :
		TYPE_INVALID;

	if (type == TYPE_INVALID) { mg_send_http_error(conn, 400, "Unsupported tile type: %s", token_type.c_str()); return; }
	
	long sx = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
	long sy = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
	long sz = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
	long x = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
	long y = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
	long z = strtol(request_tokens[token_index++].c_str(), nullptr, 10);

	//mg_printf(conn, "Tile: %ld %ld %ld, %ld %ld %ld", x, y, z, sx, sy, sz);

	BoxResult &br = getBox(x, y, z, sx, sy, sz, true);

    /*
    amf::Serializer serializer;
    serializer << amf::AmfVector<unsigned int>(blocks);
    serializer << amf::AmfVector<unsigned int>(columns);

    amf::v8 arrays = serializer.data();

    if (!br.data) br.data = new amf::v8();
    amf::v8 *data = br.data;

    const int size_int = 4;
    size_t dataSize = 3 * size_int + arrays.size() + 1 * size_int;
    data->resize(dataSize);
    vassert(data->size() == dataSize, "%d %d", (int)data->size(), dataSize);

    amf::u8 *p = data->data();
    writeInt(p, bx); p += size_int;
    writeInt(p, by); p += size_int;
    writeInt(p, bz); p += size_int;
    memcpy(p, arrays.data(), arrays.size()); p += arrays.size();
    writeInt(p, maxHeight); p += size_int;
    */


    dtimerInit(timer_deserialize, "deserialization");

    amf::Deserializer deserializer;
    
    constexpr int size_int = 4;
    int bx, by, bz, maxHeight;

    auto it = br.data->cbegin();

    int datasize = br.data->size();
   
    bx = readInt(&*it); std::advance(it, size_int);
    by = readInt(&*it); std::advance(it, size_int);
    bz = readInt(&*it); std::advance(it, size_int);

    int sxyz = sx*sy*sz;
    int sxz = sx*sz;
    
    auto blocks_end = it;
    std::advance(blocks_end, sxyz*size_int);
    int size = std::distance(it, blocks_end);
    int vsize = br.data->size();
    int toend = std::distance(it, br.data->cend());
    size_t tsize = static_cast<size_t>(blocks_end - it);
    amf::AmfVector<unsigned int> amfblocks = deserializer.deserialize(it, br.data->cend()).as<amf::AmfVector<unsigned int>>();
    amf::AmfVector<unsigned int> amfcolumns = deserializer.deserialize(it, br.data->cend()).as<amf::AmfVector<unsigned int>>();

    maxHeight = readInt(&*it); std::advance(it, size_int);

    std::vector<unsigned int>& blocks = amfblocks.values;
    std::vector<unsigned int>& columns = amfcolumns.values;

	unsigned int *pixels;
	int width = sx;
	int height = sz;

    dtimerStop(timer_deserialize);

	pixels = (unsigned int*)calloc(width*height, 4);

	vassert(pixels, "Unable to allocate memory for tile pixels");

    {
        dtimer("painting");
        for (int i = 0; i < width*height; i++) {

            int classification;

            int h = columns[i];
            int topBlockIndex = i + h*sxz;

            classification = blocks[topBlockIndex];

            unsigned int rgb = 0xFF000000;

            switch (type)
            {
            case TYPE_MAP:
            {
                MapCloud *mapCloud = nullptr;//br.cornerClouds[0];
                if (!mapCloud) {
                    classification = Classification::NONE;
                    break;
                }
                ClassificationQuery cq;

                /*
                int bx, by, bz;
                getIndexBlock(topBlockIndex, bx, by, bz, sx, sz);
                Point coords;
                getCoordsFromBlock(br.bounds_tl, bx, by, bz, coords);
                cq.x = coords.x;
                cq.y = coords.y;
                cq.corners = br.cornerClouds;
                cq.lidar = static_cast<Classification>(classification);
                cq.flags = ClassificationFlags::DEFAULT;
                classification = mapCloud->getSpecializedClassification(cq);
                */

                //rgb = MapCloud::getMapColor(br.cornerClouds, coords.x, coords.y);


                //rgb = (0xFFFFFF/2) + (int)((cq.y - origin.y())/10);

                // Coord debug
                //rgb = (bx << 8) | bz;
                //rgb = (bx << 16) | (by << 8) | bz;
                //rgb = ((0xFF/2 + (mapCloud->lat - 462)*50) << 16) | ((0xFF/2 + (mapCloud->lon - 101)*50) << 8);
                //rgb = ((0xFF / 2 + (br.bx) * 50) << 8) | (0xFF / 2 + (br.bz) * 50);

                //rgb = ((int)(cq.x - origin.x()) << 8) | (int)(cq.y - origin.y());

                //rgb = (int)(cq.x - origin.x());

                //rgb = (int)((cq.y - origin.y()));

                //rgb = (0xFFFFFF/2) + (br.bounds_tl.x() - origin.x());
                //rgb = (0xFFFFFF/2) + (br.bounds_tl.y() - origin.y());
            }
            break;
            default:
                break;
            }

            if (rgb == 0xFF000000) {
                switch (classification)
                {
                case Classification::GROUND:
                    rgb = 0x964B00;
                    break;
                case Classification::VEGETATION_LOW:
                    rgb = 0x11772d;
                    break;
                case Classification::VEGETATION_MEDIUM:
                    rgb = 0x12ae3e;
                    break;
                case Classification::VEGETATION_HIGH:
                    rgb = 0x08d542;
                    break;
                case Classification::BUILDING:
                    rgb = 0xB2ACAB;
                    break;
                case Classification::BUILDING_ROOF_TILED_ORANGE:
                    rgb = 0xEA8825;
                    break;
                case Classification::BUILDING_ROOF_TILED_GRAY:
                    rgb = 0x646d6c;
                    break;
                case Classification::BUILDING_ROOF_FLAT:
                    rgb = 0xe0e0e0;
                    break;
                case Classification::GROUND_ASPHALT:
                    rgb = 0x4a607b;
                    break;
                case Classification::GROUND_CONCRETE:
                    rgb = 0x75b6bc;
                    break;
                case Classification::LOW_POINT:
                    rgb = 0x252323;
                    break;
                case Classification::SHADOW:
                    rgb = 0x101010;
                    break;
                case Classification::WATER:
                    rgb = 0x0481ec;
                    break;
                case Classification::UNASSIGNED:
                    rgb = 0x100000;
                    break;
                case Classification::NONE:
                    rgb = 0x001000;
                    break;
                default:
                    rgb = 0x000000;
                    break;
                }
            }

            rgb = (rgb << 8) | 0xFF;

            rgb = htonl(rgb);

            pixels[i] = rgb;
        }
    }

	br.mutex.unlock();

    MapImage img;
    {
        dtimer("png encode");
        encodeImage(&img, pixels, width, height);
    }

    free(pixels);

    int status;
    {
        dtimer("tile send");
        sendLiveImageHeader(conn, img.size);
	    status = mg_write(conn, img.data, img.size);
    }
	if (status == -1) plog("Tile write error: %d", status);
}

void GKOTHandleDashboardBoxes(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
    dtimer("dash boxes");
    MapImage img;
    renderBoxesUsed(img);
    sendLiveImageHeader(conn, img.size);
    int status = mg_write(conn, img.data, img.size);
    if (status == -1) plog("Debug box write error");
}

void GKOTHandleDashboardMapClouds(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
    dtimer("dash map clouds");
    MapImage img;
    renderMapClouds(img);
    sendLiveImageHeader(conn, img.size);
    int status = mg_write(conn, img.data, img.size);
    if (status == -1) plog("Debug box write error");
}

void GKOTHandleDashboardStats(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
    dtimer("dash stats");
    runtimeCounters.count();
    ujson::value json = to_json(runtimeCounters);
    std::string str = ujson::to_string(json);
    sendLiveJSONHeader(conn, str.size());
    mg_write(conn, str.data(), str.size());
}

void GKOTHandleDebug(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{

    mg_printf(conn, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
    mg_printf(conn, "<html><body>");
    mg_printf(conn, "Hello!<br><pre>");

    LASreadOpener lasreadopener;
    lasreadopener.set_file_name("W:/gis/arso/laz/gkot/b_35/D96TM/TM_462_101.laz");
    LASreader* lasreader = lasreadopener.open();

    F64 min_x = lasreader->get_min_x();
    F64 min_y = lasreader->get_min_y();
    F64 min_z = lasreader->get_min_z();
    F64 max_x = lasreader->get_max_x();
    F64 max_y = lasreader->get_max_y();
    F64 max_z = lasreader->get_max_z();

    int xs = (int)floor(min_x);
    int xe = ((int)floor(max_x)) + 1;
    int ys = (int)floor(min_y);
    int ye = ((int)floor(max_y)) + 1;
    int zs = (int)floor(min_z * 100);
    int ze = ((int)floor(max_z * 100)) + 1;

    int mx = xe - xs;
    int my = ye - ys;
    int mz = ze - zs;

    int scale = 1;

    int mw = my / scale;
    int mh = mx / scale;

    mg_printf(conn, "%s\n", info->request_uri);
    mg_printf(conn, "min x: %f  y: %f  z: %f\n", min_x, min_y, min_z);
    mg_printf(conn, "max x: %f  y: %f  z: %f\n", max_x, max_y, max_z);
    mg_printf(conn, "x %d -> %d  y: %d -> %d\n", xs, xe, ys, ye);
    mg_printf(conn, "map: %d x %d x %d\n", my, mx, mz);


    int *map = (int*)calloc(sizeof(int)*mw*mh, 1);

    int count = 0;
    while (lasreader->read_point()) {
        LASpoint *point = &lasreader->point;
        int bx = (point->Y / 100 - ys) / scale;
        int by = (point->X / 100 - xs) / scale;
        int bz = point->Z - zs;
        int index = bx + by*mw;
        int v = map[index];
        int vz = v & 0xFFFF;
        if (bz > vz) {
            map[index] = (point->classification << 16) | bz;
        }
        //if (count < 100) mg_printf(conn, "%d, %d, %d -> %d\n", bx, by, v, bz);
        //if (count < 100) mg_printf(conn, "%d, %d, %d classification %d\n", bx, by, bz, point->classification);
        count++;
        //if (count > 500000) break;
    }
    mg_printf(conn, "%d\n", count);

    lasreader->close();
    delete lasreader;

    vassert(mw > 0 && mh > 0, "Invalid size");

    char *pixels = (char*)malloc(mw*mh * 3);
    assert(pixels);
    for (int i = 0; i < mw*mh; i++) {
        int v = map[i];
        int vz = v & 0xFFFF;
        int vclass = (v >> 16) & 0xFF;
        //if (i < 100) mg_printf(conn, "%d %d %d %d\n", i, v, zs, ze);
        vz = vz * 0xFF / (ze - zs);
        int pi = i * 3;
        int r, g, b;
        switch (vclass)
        {
        case 2: // Ground
            r = vz;
            g = 0;
            b = 0;
            break;
        case 3: // Low vegetation
            r = 0;
            g = vz;
            b = 0;
            break;
        case 4: // Medium vegetation
            r = 0;
            g = vz;
            b = 0;
            break;
        case 5: // High vegetation
            r = 0;
            g = vz;
            b = 0;
            break;
        case 6: // Building
            r = vz;
            g = 0;
            b = 0;
            break;
        case 7: // Low point
            r = 0;
            g = 0;
            b = 0x30;
            break;
        case 9: // Water
            r = 0;
            g = 0;
            b = 0xFF;
            break;
        default:
            if (vclass > 1 && i < 10000000) mg_printf(conn, "%d ", vclass);
            r = g = b = vz;
            break;
        }
        pixels[pi + 0] = r;
        pixels[pi + 1] = g;
        pixels[pi + 2] = b;
    }

    free(map);

    mg_printf(conn, "</pre>");

    MapImage img;
    stbi_write_png_to_func(&writeToBuffer, &img, mw, mh, 3, pixels, 0);

    free(pixels);

    std::string pngDataURI;

    base64::encoder encoder;
    std::istringstream pngIn = std::istringstream(std::string(static_cast<const char*>(img.data), img.size));
    std::ostringstream pngOut;
    encoder.encode(pngIn, pngOut);
    pngDataURI = "data:image/png;base64," + pngOut.str();

    mg_printf(conn, "<img width=\"%d\" height=\"%d\" alt=\"map\" src=\"%s\" />", mw, mh, pngDataURI.c_str());


    mg_printf(conn, "</body></html>\n");
}


void GKOTHandleDebugBoxes(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
	mg_printf(conn, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
	mg_printf(conn, "<html><body>");
	mg_printf(conn, "Box debug!<pre>");

	MapCloud *mc = getMapCloud(462, 101);

	int mw = 1000;
	int mh = mw;

	PointCloud all(50);
	PointCloud ground(20);

	mc->load(&all, &ground);

    Vec origin;
    origin << 462000, 101000, 0;

	int *classMap = (int*)calloc(sizeof(int)*mw*mh, 1);

	int count = 0;
	size_t pointNum = all.getPointNum();
	for (size_t i = 0; i < pointNum; i++) {
		Point &p = all.getPoint(i);
		int bx, by, bz;
		getBlockFromCoords(origin, p, bx, by, bz);
		int index = getBlockIndex(bx, 0, bz, mw, mw*mh);
		int v = classMap[index];
		int vy = v & 0xFFFF;
		if (by > vy) {
			classMap[index] = (p.classification << 16) | by;
		}
		count++;
	}

	mg_printf(conn, "%d points\n", count);

	mg_printf(conn, "</pre>");

	unsigned int *pixels;
	int width = mw;
	int height = mh;

	pixels = (unsigned int*)calloc(width*height, 4);

	vassert(pixels, "Unable to allocate memory for pixels");
	for (int i = 0; i < mw*mh; i++) {
		int v = classMap[i];
		int vz = v & 0xFFFF;
		Classification vclass = static_cast<Classification>((v >> 16) & 0xFF);

		double x = origin[0] + (i % mw);
		double y = origin[1] + (i / mw);

		unsigned int rgb;

		switch (vclass)
		{
		case Classification::GROUND:
			rgb = 0x964B00;
			break;
		case Classification::VEGETATION_LOW:
			rgb = 0x11772d;
			break;
		case Classification::VEGETATION_MEDIUM:
			rgb = 0x12ae3e;
			break;
		case Classification::VEGETATION_HIGH:
			rgb = 0x08d542;
			break;
		case Classification::BUILDING:
			rgb = 0xB2ACAB;
			break;
		case Classification::BUILDING_ROOF_TILED_ORANGE:
			rgb = 0xEA8825;
			break;
		case Classification::BUILDING_ROOF_TILED_GRAY:
			rgb = 0x586261;
			break;
		case Classification::BUILDING_ROOF_FLAT:
			rgb = 0xC8C8C8;
			break;
		case Classification::GROUND_ASPHALT:
			rgb = 0x626C78;
			break;
		case Classification::GROUND_CONCRETE:
			rgb = 0xB5B1A8;
			break;
		case Classification::LOW_POINT:
			rgb = 0x433F40;
			break;
		case Classification::SHADOW:
			rgb = 0x101010;
			break;
		case Classification::WATER:
			rgb = 0x0481ec;
			break;
		default:
			rgb = 0x000000;
			break;
		}

		rgb = (rgb << 8) | 0xFF;

		rgb = htonl(rgb);

		pixels[i] = rgb;
	}

	printImage(conn, pixels, width, height);
	free(pixels);

	mg_printf(conn, "</body></html>\n");
}


void DOF84HandleDebug(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
    mg_printf(conn, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
    mg_printf(conn, "<html><body>");
    mg_printf(conn, "Hello!<pre>");
    
    MapCloud *mc = getMapCloud(462, 101);

    int mw = 1000;
    int mh = mw;

    PointCloud all(50);
    PointCloud ground(20);

    mc->load(&all, &ground);

    Vec origin;
    origin << 462000, 101000, 0;

    unsigned char *map = static_cast<unsigned char*>(mc->map.data);
    assert(map);

    int *classMap = (int*)calloc(sizeof(int)*mw*mh, 1);

    int count = 0;
    size_t pointNum = all.getPointNum();
    for (size_t i = 0; i < pointNum; i++) {
        Point &p = all.getPoint(i);
        int bx, by, bz;
        getBlockFromCoords(origin, p, bx, by, bz);
        //int index = bx + bz*mw;
        int index = getBlockIndex(bx, 0, bz, mw, mw*mh);
        int v = classMap[index];
        int vy = v & 0xFFFF;
        if (by > vy) {
            classMap[index] = (p.classification << 16) | by;
        }
        //if (count < 100) mg_printf(conn, "%d, %d, %d -> %d\n", bx, by, v, bz);
        //if (count < 100) mg_printf(conn, "%d, %d, %d classification %d\n", bx, by, bz, point->classification);
        count++;
        //if (count > 500000) break;
    }

    mg_printf(conn, "%d points\n", count);

    mg_printf(conn, "</pre>");

    unsigned int *pixels;
    int width = mw;
    int height = mh;

    pixels = (unsigned int*)calloc(width*height, 4);

    vassert(pixels, "Unable to allocate memory for pixels");
    for (int i = 0; i < mw*mh; i++) {
        int v = classMap[i];
        int vz = v & 0xFFFF;
        Classification vclass = static_cast<Classification>((v >> 16) & 0xFF);
        //if (i < 100) mg_printf(conn, "%d %d %d %d\n", i, v, zs, ze);

        double x = origin[0] + (i % mw);
        double y = origin[1] + (i / mw);

        //vclass = classifyPixel(pixel, static_cast<Classification>(vclass));

		//MapCloud* corners[4] = { mc, nullptr, nullptr, nullptr };
		//ClassificationQuery cq = { x, y, static_cast<MapCloud**>(&corners), vclass, ClassificationFlags::DEFAULT };
        //vclass = mc->getSpecializedClassification(cq);

        unsigned int rgb;

        switch (vclass)
        {
        case Classification::GROUND:
            rgb = 0x964B00;
            break;
        case Classification::VEGETATION_LOW:
            rgb = 0x11772d;
            break;
        case Classification::VEGETATION_MEDIUM:
            rgb = 0x12ae3e;
            break;
        case Classification::VEGETATION_HIGH:
            rgb = 0x08d542;
            break;
        case Classification::BUILDING:
            rgb = 0xB2ACAB;
            break;
        case Classification::BUILDING_ROOF_TILED_ORANGE:
            rgb = 0xEA8825;
            break;
        case Classification::BUILDING_ROOF_TILED_GRAY:
            rgb = 0x586261;
            break;
        case Classification::BUILDING_ROOF_FLAT:
            rgb = 0xC8C8C8;
            break;
        case Classification::GROUND_ASPHALT:
            rgb = 0x626C78;
            break;
        case Classification::GROUND_CONCRETE:
            rgb = 0xB5B1A8;
            break;
        case Classification::LOW_POINT:
            rgb = 0x433F40;
            break;
        case Classification::SHADOW:
            rgb = 0x101010;
            break;
        case Classification::WATER:
            rgb = 0x0481ec;
            break;
        default:
            rgb = 0x000000;
            break;
        }

        //rgb = mc->getSpecializedClassification(cq);

        rgb = (rgb << 8) | 0xFF;

        rgb = htonl(rgb);

        pixels[i] = rgb;
    }

    printImage(conn, pixels, width, height);
    free(pixels);

    mg_printf(conn, "</body></html>\n");
}


int
MainHandler(struct mg_connection *conn, void *cbdata)
{
    //mapCloudHash.mutex.lock();

    const mg_request_info *info = mg_get_request_info(conn);

    //bool inside = lasreader->inside_rectangle(462000.00, 101000.00, 462000.00 + 250, 101000.00 + 600);

    //const char* status;

    if (strcmp(info->request_uri, "/gkot/box") == 0) {
		GKOTHandleBox(conn, cbdata, info);
	} else if (strcmp(info->request_uri, "/dashboard/") == 0) {
		mg_send_file(conn, "../../www/dashboard/dashboard.html");
    } else if (strcmp(info->request_uri, "/dashboard/boxes.png") == 0) {
        GKOTHandleDashboardBoxes(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/dashboard/mapClouds.png") == 0) {
        GKOTHandleDashboardMapClouds(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/dashboard/stats.json") == 0) {
        GKOTHandleDashboardStats(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/gkot/debug") == 0) {
        GKOTHandleDebug(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/gkot/debugBoxes") == 0) {
		GKOTHandleDebugBoxes(conn, cbdata, info);
	} else if (startsWith(info->request_uri, "/debug/tile_") && endsWith(info->request_uri, ".png")) {
		GKOTHandleTile(conn, cbdata, info);
	} else if (strcmp(info->request_uri, "/debug/") == 0) {
		mg_send_file(conn, "../../www/debug/debug.html");
	} else if (strcmp(info->request_uri, "/dof84") == 0) {
		DOF84HandleDebug(conn, cbdata, info);
    } else if (startsWith(info->request_uri, "/")) {
		if (strstr(info->request_uri, "..") != NULL) return 0;
		char *path;
		vassert(asprintf(&path, "../../www/%s", info->request_uri) != -1, "Unable to allocate memory for path string");
		mg_send_file(conn, path);
	} else {
        return 0;
    }

    ++requestsServed;

    plog("%s %s?%s", info->request_method, info->request_uri, info->query_string);
    
    //printf("%s %s %s %s\n", info->request_method, info->request_uri, info->query_string, status);

    //mapCloudHash.mutex.unlock();

    return 1;
}

int main(int argc, char *argv[])
{

    memset(blockToColor, 0, sizeof(blockToColor));
    blockToColor[0x023] = 0xE0E0E0;
    blockToColor[0x123] = 0xEB833C;
    blockToColor[0x223] = 0xC65DD0;
    blockToColor[0x323] = 0x5980D0;
    blockToColor[0x423] = 0xC9BB1D;
    blockToColor[0x523] = 0xD6C821;
    blockToColor[0x623] = 0xE19CAF;
    blockToColor[0x723] = 0x3F3F3F;
    blockToColor[0x823] = 0xA3AAAA;
    blockToColor[0x923] = 0x29799A;
    blockToColor[0xA23] = 0x8635CC;
    blockToColor[0xB23] = 0x27349C;
    blockToColor[0xC23] = 0x51301A;
    blockToColor[0xD23] = 0x3A5019;
    blockToColor[0xE23] = 0xA32C28;
    blockToColor[0xF23] = 0x181414;

    Vec test{ 462000, 101000, 200 };
	Point src, dst;
	src.x = test.x() + 123;
	src.y = test.y() + 456;
	src.z = test.z() + 67;
	int bx, by, bz;
	getBlockFromCoords(test, src, bx, by, bz);
	getCoordsFromBlock(test, bx, by, bz, dst);

	vassert(src.x == dst.x, "Block transformation test failed for X");
	vassert(src.y == dst.y, "Block transformation test failed for Y");
	vassert(src.z == dst.z, "Block transformation test failed for Z");

    vassert(fishnet.load(fishnetPath), "Unable to open fishnet database: %s", fishnetPath);

    const char *options[] = {
        "listening_ports", PORT,
        "request_timeout_ms", "10000",
        "error_log_file", "error.log",
#ifdef USE_WEBSOCKET
        "websocket_timeout_ms", "3600000",
#endif
#ifndef NO_SSL
        "ssl_certificate", "../../resources/cert/server.pem",
#endif
        0
    };


    struct mg_callbacks callbacks;
    struct mg_context *server;
    struct mg_server_ports ports[32];
    int portCount, n;
    int err = 0;


    /* Check if libcivetweb has been built with all required features. */
#ifdef USE_IPV6
    if (!mg_check_feature(8)) {
        fprintf(stderr,
            "Error: Embedded example built with IPv6 support, "
            "but civetweb library build without.\n");
        err = 1;
    }
#endif
#ifdef USE_WEBSOCKET
    if (!mg_check_feature(16)) {
        fprintf(stderr,
            "Error: Embedded example built with websocket support, "
            "but civetweb library build without.\n");
        err = 1;
    }
#endif
#ifndef NO_SSL
    if (!mg_check_feature(2)) {
        fprintf(stderr,
            "Error: Embedded example built with SSL support, "
            "but civetweb library build without.\n");
        err = 1;
    }
#endif
    if (err) {
        fprintf(stderr, "Cannot start CivetWeb - inconsistent build.\n");
        return EXIT_FAILURE;
    }



    /* Start CivetWeb web server */
    memset(&callbacks, 0, sizeof(callbacks));
    server = mg_start(&callbacks, 0, options);

    mg_set_request_handler(server, "/", MainHandler, 0);

    /* List all listening ports */
    memset(ports, 0, sizeof(ports));
    portCount = mg_get_server_ports(server, 32, ports);

    for (n = 0; n < portCount && n < 32; n++) {
        const char *proto = ports[n].is_ssl ? "https" : "http";
        const char *host = "";

        if ((ports[n].protocol & 1) == 1) {
            /* IPv4 */
            host = "127.0.0.1";
        }

        if ((ports[n].protocol & 2) == 2) {
            /* IPv6 */
            host = "[::1]";
        }

        plog("Server up at %s://%s:%i/", proto, host, ports[n].port);
        plog("");
    }


    /*
    HANDLE hPipe;
    DWORD dwWritten;

    hPipe = CreateFile(
        TEXT("\\\\.\\pipe\\geomvis"),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );
    */

    /*
    hPipe = CreateNamedPipeA(
        "\\\\.\\pipe\\geomvis",
        PIPE_ACCESS_DUPLEX,
        PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_REJECT_REMOTE_CLIENTS,
        PIPE_UNLIMITED_INSTANCES,
        4096,
        4096,
        0,
        NULL
    );


    if (hPipe == INVALID_HANDLE_VALUE) {
        printf("Pipe creation failed: %ld", GetLastError());
    }

    if (hPipe != INVALID_HANDLE_VALUE)
    {
        printf("Pipe created\n");

        bool success = false;

        while (!success) {
            success = WriteFile(hPipe,
                "Hello Pipe\n",
                12,   // = length of string + terminating '\0' !!!
                &dwWritten,
                NULL);

            printf("written %d %ld %ld\n", success, dwWritten, GetLastError());
            Sleep(1000);
        }

        CloseHandle(hPipe);
    }
    */

    //getMapCloud(462, 101);
    //getMapCloud(461, 101);

    /* Wait until the server should be closed */
    while (!exitNow) {
#ifdef _WIN32
        Sleep(1000);
#else
        sleep(1);
#endif
    }

    /* Stop the server */
    mg_stop(server);
    plog("Server stopped.");
    plog("Bye!");

    return EXIT_SUCCESS;

}



/*
const char *delim = "&";
char *tok = info->query_string ? _strdup(info->query_string) : NULL;
char *param = tok ? strtok(tok, delim) : NULL;
while (param) {
char *equals = strchr(param, '=');
if (equals) {
parseParamLong("x", x, param, equals);
parseParamLong("y", y, param, equals);
parseParamLong("z", z, param, equals);
parseParamLong("w", sx, param, equals);
parseParamLong("h", sz, param, equals);
parseParamLong("sx", sx, param, equals);
parseParamLong("sy", sy, param, equals);
parseParamLong("sz", sz, param, equals);
} else {
parseParamDef("debug", debug, param);
}
param = strtok(NULL, delim);
}
*/
//*
//LASreader* lasreader = getLAS(462, 101);

// spatial index? lasreader->set_index(...);

/*
F64 min_x = lasreader->get_min_x();
F64 min_y = lasreader->get_min_y();

// x -> z
// y -> x
// z -> y

int xs = (int)floor(min_y) + x;
int zs = (int)floor(min_x) + z;

lasreader->seek(0);
lasreader->inside_none();
lasreader->seek(0);

bool inside = lasreader->inside_rectangle(zs, xs, zs + sz, xs + sx);
assert(inside);

F64 max_y = lasreader->get_max_y();
F64 max_x = lasreader->get_max_x();
int xe = ((int)floor(max_y)) + 1;
int ze = ((int)floor(max_x)) + 1;

F64 min_z = lasreader->get_min_z();
F64 max_z = lasreader->get_max_z();
int ys = (int)floor(min_z * 100);
int ye = ((int)floor(max_z * 100)) + 1;

debugPrint("min x: %f  y: %f  z: %f\n", min_x, min_y, min_z);
debugPrint("max x: %f  y: %f  z: %f\n", max_x, max_y, max_z);

debugPrint("x %d -> %d\n", xs, xe);
debugPrint("y %d -> %d\n", ys, ye);
debugPrint("z %d -> %d\n", zs, ze);


// YZX
// uint element value
// 0x000000FF	block id
// 0x00000F00	block data
// 0x0000F000	skylight
// 0x000F0000	blocklight
// 0x0FF00000	? height of column (stored at y = 0)
//
// const returned, not stored
// 0xFFFFFFFF	not available yet (delayed)

//unsigned int *blocks = (unsigned int*)calloc(sx*sy*sz, sizeof(unsigned int));
//amfblocks.values.assign(blocks, blocks + sx*sy*sz);
//serializer << amfblocks;


while (lasreader->read_point()) {
LASpoint *point = &lasreader->point;
int bx = (point->Y / 100) - xs;
int bz = (point->X / 100) - zs;
int by = (point->Z / 100) - y - 200;
int index = bx + bz*sx + by*sxz;
*/

//*/

/*
while (cloudpoints...) {

if (by < 0 || by > sy) continue;

int bid = 12;
switch (point->classification)
{
case 1: bid = 13; break; // Unassigned -> Gravel
case 2: bid = 43; break; // Ground -> Double Stone Slab
case 3:
case 4: bid = 3; break; // Low/Medium Vegetation -> Dirt
case 6: bid = 98; break; // Building -> Stone Bricks
case 5: bid = 18; break; // High Vegetation -> Leaves
case 9: bid = 9; break; // Water -> Still Water
}

blocks[index] = bid;

int colindex = bx + bz*sx;
int col = columns[colindex];
if (by > col) columns[colindex] = by;
if (by > maxHeight) maxHeight = by;

//if (count < 100) debugPrint("%d, %d, %d  %d\n", bx, by, bz, index);

//int index = bx + by*w;
//if (count < 100) debugPrint("%d, %d, %d -> %d\n", bx, by, v, bz);
//if (count < 100) debugPrint("%d, %d, %d classification %d\n", bx, by, bz, point->classification);
count++;
//if (count > 500000) break;
}

//*/

/*
for (int bz = 0; bz < sz; bz++) {
for (int bx = 0; bx < sx; bx++) {
for (int by = 0; by < 5; by++) {
int index = bx + bz*sx + by*sxz;
blocks[index] = 3;

int colindex = bx + bz*sx;
int col = columns[colindex];
if (by > col) columns[colindex] = by;
if (by > maxHeight) maxHeight = by;
}
}
}
*/
