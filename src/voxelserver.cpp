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
#include <random>
#include <condition_variable>
#include <thread>

#include <sys/types.h>
#include <sys/stat.h>

#pragma warning(push)
#pragma warning(disable:4996 4267)

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

#include "lz4/lz4.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <Eigen/Core>

#include "optionparser.h"
#include "fmt/format.h"
#include "mappedfile/mappedfile.h"

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

typedef double pcln;
typedef Eigen::Vector3d Vec;

//static const int defaultPower = 11;
static const int defaultPower = 14;
static const int defaultMapMemoryLimit = 1000;
static int mapMemoryLimit;

static const char* nameFormat = "{0}_{1}";

static const char* defaultPort = "8888";
static const char* defaultPath = ".";

static const char* webRel = "www/";
static const char* fishnetRel = "fishnet/LIDAR_FISHNET_D96.dbf";
static const char* gkotRel = "laz/gkot";
static const char* dof84Rel = "dof84";
static const char* bdmrRel = "bdmr";

static const char* gkotFormat = "{0}/D96TM/TM_{1}.laz";
static const char* dof84Format = "{0}/{1}.png";
static const char* bdmrFormat = "{0}/D96TM/TM1_{1}.bin";

static const int bdmrWidth = 1001;
static const int bdmrHeight = 1001;

static const int mapTileWidth = 1000;
static const int mapTileHeight = 1000;

static const pcln seaThreshold = 0.1;
static const pcln waterMaxDepth = 20;

static std::string gkotFullFormat;
static std::string dof84FullFormat;
static std::string bdmrFullFormat;
static std::string webPath;
static std::string fishnetPath;


// Ljubljana
//Vec default_origin{ 462000, 101000, 200 };

// Bled
//Vec origin{ 430350, 135776, 400 };
//Vec origin{ 431785, 136409, 400 };

// Smarna
//Vec origin{ 458937, 109675, 300 };

// Piran
//Vec origin{ 388143, 43887, 0 };
Vec default_origin{ 388000, 44000, 0 };
//Vec default_origin{ 387000, 44000, 0 };

// Triglav
//Vec origin{ 410750, 137541, 2550 };
//Vec origin{ 410488, 138284, 2500 };
//Vec origin{ 410488, 138284, 2000 };


bool exitNow = 0;

#define vassert(x, format, ...) if (!(x)) { printf(format, __VA_ARGS__); __debugbreak(); }

static std::atomic<int> plogLevel = { 0 };
#define plog(format, ...) printf("%*s" format "\n", plogLevel*2, " ", __VA_ARGS__)

struct PlogScope
{
    PlogScope() { plogLevel++; }
    ~PlogScope() { plogLevel--; }
};
#define plogScope() PlogScope plog_scope_struct



/*
static const char* otrPathFormat = "W:/gis/arso/laz/otr/b_35/D96TM/TMR_%d_%d.laz";
static const char* nanoflannAllPathFormat = "W:/gis/arso/nanoflann/gkot/b_35/D96TM/TM_%d_%d.kdz";
static const char* nanoflannGroundPathFormat = "W:/gis/arso/nanoflann/otr/b_35/D96TM/TM_%d_%d.kdtree";
*/


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
    OVERLAP                    = 12,
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

    { Classification::WATER,{
        Classification::WATER
    } },

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
    */
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

static void slashToBackslash(char *str) {
    size_t len = strlen(str);
    for (size_t i = 0; i < len; i++) {
        char &c = str[i];
        if (c == '/') c = '\\';
    }
}

static void normalizeSlashes(char *str) {
    slashToBackslash(str);
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
    STATP,
    EXEC_TIME
};

enum RuntimeCounterUnit {
    COUNT,
    BYTES
};

class Timer;

typedef long long Count;
class RuntimeCounter {
    std::atomic<Count> count;

public:
    RuntimeCounterType type;
    RuntimeCounterUnit unit;
    const char *id;
    const char *name;

    Timer *timer;

    RuntimeCounter(const char *id, const char *name, RuntimeCounterType type = RuntimeCounterType::STAT, RuntimeCounterUnit unit = RuntimeCounterUnit::COUNT)
        : type(type), unit(unit), id(id), name(name), timer(nullptr), count(0) {
        runtimeCounters.add(this);
    }

    RuntimeCounter& operator++() { ++count; return *this; }
    RuntimeCounter& operator--() { --count; return *this; }
    RuntimeCounter& operator+=(Count c) { count += c; return *this; }
    RuntimeCounter& operator-=(Count c) { count -= c; return *this; }

    Count load() {
        Count c = count.load();
        if (type != STATP) count = 0;
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


#define ADD_COUNTER(id, name, ...) RuntimeCounter id (#id, name, __VA_ARGS__);

ujson::value to_json(RuntimeCounters const &rcs) {
    auto arr = ujson::array();
    for (auto &iter : rcs.list) {
        arr.push_back(
            ujson::object {
                { "type", iter->type },
                { "unit", iter->unit },
                { "id", iter->id },
                { "name", iter->name },
                { "value", static_cast<double>(iter->load()) }
            }
        );
    }
    return arr;
}


ADD_COUNTER(boxesSent, "Boxes sent");
ADD_COUNTER(boxesCreated, "Boxes created");
ADD_COUNTER(pointsLoaded, "Points loaded");
ADD_COUNTER(requestsServed, "Requests served");
ADD_COUNTER(boxesCached, "Boxes cached", RuntimeCounterType::STATP);
ADD_COUNTER(boxCacheBytes, "Box cache memory", RuntimeCounterType::STATP, RuntimeCounterUnit::BYTES);
ADD_COUNTER(mapOrthoBytes, "Map ortho memory", RuntimeCounterType::STATP, RuntimeCounterUnit::BYTES);
ADD_COUNTER(mapCloudsInUse, "Map clouds in use", RuntimeCounterType::STATP);
ADD_COUNTER(mapCloudsLoaded, "Map clouds loaded", RuntimeCounterType::STATP);



#if __cplusplus < 201103L && (!defined(_MSC_VER) || _MSC_VER < 1700)
#error Timer requires C++11
#elif 1
#include <chrono>
class Timer {
    typedef std::chrono::high_resolution_clock clock;
    typedef std::chrono::nanoseconds ns;

    std::mutex mutex;

    clock::time_point start;
    long long counted;

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
            counter = new RuntimeCounter(name, name, RuntimeCounterType::EXEC_TIME);
        }
        counter->timer = this;
    }
    ~Timer()
    {
        count();
        counter->timer = nullptr;
    }

    unsigned long long getCountedMicro()
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
        long long c = tock().count() / 1000L;
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
        plog("%s %*s%lld%s", name, (int)(20 - strlen(name)), " ", c, unit);
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

long getParamLong(const char *queryString, size_t queryLength, const char *name, long defaultValue = 0)
{
    char param[20];
    int ret = mg_get_var(queryString, queryLength, name, param, sizeof(param));
    return ret < 0 ? defaultValue : strtol(param, NULL, 10);
}

uint32_t getParamUInt(const char *queryString, size_t queryLength, const char *name, uint32_t defaultValue = 0)
{
    char param[20];
    int ret = mg_get_var(queryString, queryLength, name, param, sizeof(param));
    return ret < 0 ? defaultValue : strtoul(param, NULL, 10);
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
        int ret = _mkdir(copy.c_str());
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

void writeUInt(amf::u8 *p, uint32_t v)
{
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >> 8) & 0xFF;
    p[3] = v & 0xFF;
}

amf::u8* writeUIntVector(amf::u8 *p, const std::vector<unsigned int> &vec)
{
    size_t len = vec.size();
    writeUInt(p, (uint32_t)len);
    p += 4;
    for (size_t i = 0; i < len; i++) {
        writeUInt(&p[i << 2], vec[i]);
    }
    p += len << 2;
    return p;
}

void writeInt(amf::u8 *p, int v)
{
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >> 8) & 0xFF;
    p[3] = v & 0xFF;
}

uint32_t readUInt(const amf::u8 *p)
{
    return (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
}

int readInt(const amf::u8 *p)
{
    return (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
}

const amf::u8* readUIntVector(const amf::u8 *p, std::vector<unsigned int> &vec)
{
    size_t len = readUInt(p);
    p += 4;
    vec.resize(len);
    for (size_t i = 0; i < len; i++) {
        vec[i] = readUInt(&p[i << 2]);
    }
    p += len << 2;
    return p;
}





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
    static const int retryNum = 6;
    static const int retryInitialSleepMs = 10;

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
        int retrySleepMs = retryInitialSleepMs;
        for (int retries = 0; retries < retryNum && !reader; retries++) {
            if (retries > 0) {
                Sleep(retrySleepMs);
                retrySleepMs *= 2;
            }
            LASreadOpener opener = LASreadOpener();
            opener.set_file_name(path);
            reader = opener.open();
            if (!reader) {
                plog("Unable to open %s, retrying %d more times after %dms", path, retryNum - 1 - retries, retrySleepMs);
            }
        }

        if (!reader) plog("Unable to open %s after %d retries", path, retryNum);
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

        reader->inside_none();
    }
};

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

static const long boxHashAccessStart = 0xFF;
static std::atomic<long> boxHashAccess = { boxHashAccessStart };
static const long long mapCloudAccessStart = 0xFF;
static std::atomic<long long> mapCloudAccess = { mapCloudAccessStart };

class MapCloud;
struct MapCloudRef;

struct ClassificationQuery {
    double x;
    double y;
    MapCloud* corners[4];
    Classification lidar;
    int flags;

    ClassificationQuery() : x(NAN), y(NAN), lidar(Classification::NONE), flags(ClassificationFlags::DEFAULT) {}
};

class MapCloud {
public:
    static const int READER_NUM = 6;
    const int lat;
    const int lon;

    bool readerFree[READER_NUM];
    PointCloudIO readers[READER_NUM];
    int readersFree = READER_NUM;

    int32_t* bdmrMap;
    size_t bdmrSize;

protected:
    const std::string lidarPath;
    const std::string mapPath;

    std::mutex mutex;
    std::condition_variable cond;

    std::atomic<long long> access;
    std::atomic<int> references;

    MapImage map;

public:

    MapCloud(int lat, int lon, std::string lidarPath, std::string mapPath, std::string bdmrPath) :
        lat(lat), lon(lon),
        lidarPath(lidarPath),
        mapPath(mapPath),
        bdmrMap(nullptr),
        bdmrSize(0),
        references(0)
    {
        ++mapCloudsLoaded;

        plogScope();

        {
            dtimer("mapcloud readers");
            for (int i = 0; i < READER_NUM; i++) {
                readerFree[i] = true;
            }
        }

        {
            dtimer("mapcloud map image");

            int reqComp = 3;
            int retComp;
            map.data = stbi_load(mapPath.c_str(), &map.width, &map.height, &retComp, reqComp);
            map.size = map.width*map.height*retComp;
            mapOrthoBytes += map.size;
            if (map.data == nullptr) {
                plog("Unable to open %s", mapPath.c_str());
            }
            else {
                assert(reqComp == retComp);
            }
        }

        {
            dtimer("mapcloud bdmr map");
            size_t length;
            char* mapped = map_file(bdmrPath.c_str(), &length);
            if (mapped == nullptr) {
                plog("Unable to open %s", bdmrPath.c_str());
            } else {
                bdmrSize = bdmrWidth * bdmrHeight * sizeof(int32_t);
                if (bdmrSize != length) {
                    plog("Invalid bdmr size, expected %zd actual %zd", bdmrSize, length);
                    bdmrSize = 0;
                }
                else {
                    bdmrMap = reinterpret_cast<int32_t*>(mapped);
                }
            }

        }
    }

    ~MapCloud() {
        --mapCloudsLoaded;

        for (int i = 0; i < READER_NUM; i++) {
            readerFree[i] = false;
            readers[i].close();
        }

        if (map.data) {
            stbi_image_free(map.data);
            mapOrthoBytes -= map.size;
            map.data = nullptr;
        }

        if (bdmrMap != nullptr) {
            unmap_file(reinterpret_cast<char*>(bdmrMap), bdmrSize);
            bdmrMap = nullptr;
        }
    }

    int getRefNum() {
        return references;
    }

    long long getAccessTime() {
        return access;
    }

    void acquire() {
        std::unique_lock<std::mutex> lock(mutex);
        ++mapCloudsInUse;
        ++references;
        access = mapCloudAccess++;
    }

    void release() {
        std::unique_lock<std::mutex> lock(mutex);
        --mapCloudsInUse;
        --references;
        access = mapCloudAccess++;
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

        reader->open(lidarPath.c_str());
        reader->load(all, ground, min_x, min_y, max_x, max_y);
        reader->close();

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

            if (dist < minDist) {
                minDist = dist;
                minClassification = iter.second;
            }
        }

        return minClassification;
    }

    void getMapCoords(const double x, const double y, int *mx, int *my) const {
        *mx = (int)(x - lat*mapTileWidth);
        *my = (int)(mapTileHeight - 1 - (y - lon*mapTileHeight));
    }

    const MapImage& getMap() {
        return map;
    }

    static MapCloud* fromCorners(MapCloud*const (&corners)[4], const double x, const double y, int *mx = nullptr, int *my = nullptr) {
        int cornerNum = 4;
        for (int i = 0; i < cornerNum; i++) {
            MapCloud* mapCloud = corners[i];
            if (!mapCloud) continue;
            int mapX, mapY;
            mapCloud->getMapCoords(x, y, &mapX, &mapY);
            if (mapX >= 0 && mapY >= 0 && mapX < mapTileWidth && mapY < mapTileHeight) {
                if (mx) *mx = mapX;
                if (my) *my = mapY;
                return mapCloud;
            }
        }
        return nullptr;
    }

    static unsigned int getMapColor(MapCloud* (&corners)[4], const double x, const double y) {
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

    static pcln getHeight(MapCloud* (&corners)[4], const double x, const double y) {
        int mx;
        int my;
        MapCloud *mapCloud = fromCorners(corners, x, y, &mx, &my);
        if (!mapCloud) {
            return NAN;
        }
        return mapCloud->getPointHeight(mx, my);
    }

    pcln getPointHeight(int mx, int my) const {
        if (mx < 0) mx = 0;
        if (my < 0) my = 0;
        if (mx >= bdmrWidth) mx = bdmrWidth - 1;
        if (my >= bdmrHeight) my = bdmrHeight - 1;

        if (bdmrMap == nullptr) {
            return NAN;
        }

        int mapIndex = mx + (mapTileHeight - 1 - my) * bdmrWidth;
        return bdmrMap[mapIndex] / (pcln)100;
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
    std::vector<T> hash;

    SpatialHash()
    {
    }

    ~SpatialHash()
    {
        if (!initialized) return;
    }

    void resize(int power)
    {
        size = 1 << power;
        mask = size - 1;
        hash.resize(size);
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

struct MapCloudRef {
    MapCloud* cloud;

    MapCloudRef(MapCloud* cloud = nullptr) : cloud(cloud) {
        if (cloud) cloud->acquire();
    }

    MapCloudRef(const MapCloudRef &ref) {
        cloud = ref.cloud;
        if (cloud) cloud->acquire();
    }

    MapCloudRef& operator=(const MapCloudRef& ref) {
        if (cloud) cloud->release();
        cloud = ref.cloud;
        if (cloud) cloud->acquire();
        return *this;
    }

    ~MapCloudRef() {
        if (cloud) cloud->release();
    }
};

static void trimMapCloudList()
{
    std::lock_guard<std::mutex> listLock(mapCloudListMutex);

    dtimer("mapcloud trim");

    int sleepMin = 50;
    int sleepMax = 2000;
    int sleepMs = sleepMin;

    long long mapMemoryLimitBytes = mapMemoryLimit * 1000000L;

    while (mapOrthoBytes.load() > mapMemoryLimitBytes) {
        MapCloud* lru = nullptr;
        long long minAccessTime = MAXLONGLONG;
        for (auto i = mapCloudList.begin(); i != mapCloudList.end(); i++) {
            MapCloud* mc = *i;
            if (mc->getRefNum() > 0) continue;
            long long access = mc->getAccessTime();
            if (access < minAccessTime) {
                minAccessTime = access;
                lru = mc;
            }
        }
        if (lru == nullptr) {
            if (sleepMs < sleepMax) sleepMs *= 2;
            if (sleepMs > sleepMax) sleepMs = sleepMax;
            plog("Waiting to free map cloud in %dms...", sleepMs);
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
        }
        else {
            sleepMs = sleepMin;
            mapCloudList.remove(lru);
            delete lru;
        }
    }
}

static unsigned int classificationToBlock(unsigned int cv)
{
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

enum BoxCompression {
    None,
    LZ4
};

struct BoxResult {
    std::mutex mutex;
    
    std::atomic<long> access;

    bool valid;

    Vec origin;
    long x, y, z;
    long sx, sy, sz;

    void *data;
    size_t dataSize;

    BoxCompression compression;
    void *compressed;
    size_t compressedSize;

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

    BoxResult() :
        data(nullptr),
        dataSize(0),

        compression(None),
        compressed(nullptr),
        compressedSize(0),

        access(0),
        valid(false) {};

    BoxResult(const BoxResult& br) {
        access = 0;
        valid = br.valid;
        x = br.x;
        y = br.y;
        z = br.z;
        sx = br.sx;
        sy = br.sy;
        sz = br.sz;
        data = br.data;
    }
    ~BoxResult() {
        removeData();
        removeCompressed();
    }

private:
    static void resizeArray(void** arr, size_t* storedSize, const size_t newSize) {
        boxCacheBytes += newSize;
        if (!*arr) {
            *arr = malloc(newSize);
        } else {
            boxCacheBytes -= *storedSize;
            *arr = realloc(*arr, newSize);
        }
        *storedSize = newSize;
    }
    static void removeArray(void** arr, const size_t size) {
        if (!*arr) return;
        boxCacheBytes -= size;
        free(*arr);
        *arr = nullptr;
    }

public:

    void* getBuffer(size_t size) {
        resizeArray(&data, &dataSize, size);
        return data;
    }

    void compress() {
        vassert(data, "Unable to compress null data");

        dtimer("box compression");

        compression = BoxCompression::LZ4;
        resizeArray(&compressed, &compressedSize, LZ4_compressBound((int)dataSize));

        int ret = LZ4_compress_default(reinterpret_cast<char*>(data), reinterpret_cast<char*>(compressed), (int)dataSize, (int)compressedSize);
        vassert(ret > 0, "Unable to LZ4 compress: %d", ret);

        resizeArray(&compressed, &compressedSize, ret);
    }

    void decompress() {
        dtimer("box decompression");

        if (data) return;

        switch (compression)
        {
        case BoxCompression::None:
            vassert(data, "Unable to decompress uncompressed data, original data not available");
            break;
        case BoxCompression::LZ4:
        {
            vassert(compressed, "Unable to LZ4 decompress null data");
            size_t actuallyNew = 0;
            resizeArray(&data, &actuallyNew, dataSize);
            int ret = LZ4_decompress_safe(reinterpret_cast<char*>(compressed), reinterpret_cast<char*>(data), (int)compressedSize, (int)dataSize);
            vassert(ret > 0, "Unable to LZ4 decompress: %d", ret)
        }
            break;
        default:
            vassert(false, "Unable to decompress, compression type unsupported: %d", compression);
        }
    }

    void removeRedundant() {
        if (compressed) removeData();
    }

    void removeCompressed() {
        removeArray(&compressed, compressedSize);
    }

    void removeData() {
        removeArray(&data, dataSize);
    }

    void send(struct mg_connection *conn) {
        mg_printf(conn,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/x-amf\r\n"
            NO_CACHE
            "\r\n"
        );
        decompress();

        vassert(data, "Unable to send box, data is null");

        mg_write(conn, data, dataSize);

        removeRedundant();
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

};


static SpatialHash<BoxResult> boxHash;
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

void sendLiveJSONHeader(struct mg_connection *conn, size_t size)
{
    mg_printf(conn,
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
        NO_CACHE
        "Content-Length: %d\r\n"
        "\r\n", size);
}




static void paintMapCloud(MapImage &raw, int lat, int lon, unsigned int color, int tileSize = 1);
static void renderMapCloudsRaw(MapImage &raw);
static void renderMapClouds(MapImage &img);


class Fishnet
{

    DBF dbf;

    int records;
    int fields;

    std::map<std::string, std::string> nameToBlock;

    MapImage clouds;

    int getFieldIndexFromName(const char *name) {
        for (int i = 0; i < fields; i++)
        {
            if (dbf.GetFieldName(i) == name) return i;
        }
        return -1;
    }

public:
    const std::string blockNotAvailable;
    double minX, minY, maxX, maxY;

    Fishnet() : blockNotAvailable("b_NA") {}

    bool paintMapClouds(MapImage &raw) {
        if (!clouds.data) return false;
        memcpy(raw.data, clouds.data, clouds.size);
        return true;
    }

    bool load(const char *path) {
        int ret = dbf.open(path);
        if (ret) {
            return false;
        }

        records = dbf.GetNumRecords();
        fields = dbf.GetNumFields();

        int colName = getFieldIndexFromName("NAME"); vassert(colName > -1, "Column not found: NAME");
        int colBlock = getFieldIndexFromName("BLOK"); vassert(colBlock > -1, "Column not found: BLOK");
        int colCenterX = getFieldIndexFromName("CENTERX"); vassert(colCenterX > -1, "Column not found: CENTERX");
        int colCenterY = getFieldIndexFromName("CENTERY"); vassert(colCenterY > -1, "Column not found: CENTERY");

        minX = INFINITY, maxX = -INFINITY;
        minY = INFINITY, maxY = -INFINITY;

        std::vector<std::pair<double, double>> coords;
        coords.resize(records);
        for (int i = 0; i < records; i++) {
            dbf.loadRec(i);
            std::string name = dbf.readField(colName); rtrim(name);
            std::string block = dbf.readField(colBlock); rtrim(block);
            std::string scenterX = dbf.readField(colCenterX); trim(scenterX);
            std::string scenterY = dbf.readField(colCenterY); trim(scenterY);
            double centerX = std::stod(scenterX);
            double centerY = std::stod(scenterY);
            minX = std::min(minX, centerX); maxX = std::max(maxX, centerX);
            minY = std::min(minY, centerY); maxY = std::max(maxY, centerY);
            nameToBlock.insert(std::make_pair(name, block));
            coords[i] = { centerX, centerY };
        }

        dbf.close();

        minX -= mapTileWidth / 2; maxX += mapTileWidth / 2;
        minY -= mapTileWidth / 2; maxY += mapTileHeight / 2;

        clouds.data = nullptr;
        renderMapCloudsRaw(clouds);
        paintRect(static_cast<unsigned int*>(clouds.data), clouds.width, 0, 0, 0x33000000, clouds.width, clouds.height);
        for (int i = 0; i < records; i++) {
            auto &pair = coords[i];
            paintMapCloud(clouds, pair.first / mapTileWidth, pair.second / mapTileHeight, 0xFFFFFFFF);
        }

        return true;
    }

    const std::string& getBlockFromName(std::string name) {
        auto it = nameToBlock.find(name);
        if (it == nameToBlock.end()) return blockNotAvailable;
        return it->second;
    }

};

Fishnet fishnet;





static void renderBoxesUsed(MapImage &img)
{
    int hashEdge = static_cast<int>(round(sqrt(boxHash.size)));

    int width = hashEdge;
    int height = width;

    int tileSize = 1;

    int pixelWidth = width*tileSize;
    int pixelHeight = height*tileSize;

    int offX = -width / 2;
    int offY = -height / 2;

    size_t pixelsLen = pixelWidth*pixelHeight;
    std::unique_ptr<unsigned int> pixels((new unsigned int[pixelsLen]()));

    for (int iy = 0; iy < height; iy++) {
        for (int ix = 0; ix < width; ix++) {
            //if (ix + iy*width >= len) break;

            BoxResult &br = boxHash.at(ix + offX, iy + offY);

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




static void paintMapCloud(MapImage &raw, int lat, int lon, unsigned int color, int tileSize)
{
    int minX = (int)floor(fishnet.minX / mapTileWidth);
    int minY = (int)floor(fishnet.minY / mapTileHeight);
    int ix = lat - minX;
    int iy = lon - minY;
    iy = raw.height/tileSize - 1 - iy;
    ix *= tileSize;
    iy *= tileSize;
    if (ix < 0 || ix + tileSize >= raw.width || iy < 0 || iy + tileSize >= raw.height) return;
    paintRect(static_cast<unsigned int*>(raw.data), raw.width,
        ix, iy,
        color,
        tileSize, tileSize
    );
}

static void renderMapCloudsRaw(MapImage &raw)
{
    int tileSize = 1;
    int tileGrid = 10;

    int minX = (int)floor(fishnet.minX / mapTileWidth);
    int minY = (int)floor(fishnet.minY / mapTileHeight);
    int width = (int)ceil((fishnet.maxX - fishnet.minX) / mapTileWidth);
    int height = (int)ceil((fishnet.maxY - fishnet.minY) / mapTileHeight);

    int pixelWidth = width * tileSize;
    int pixelHeight = height * tileSize;

    size_t pixelsLen = pixelWidth*pixelHeight;
    raw.size = pixelsLen * 4;
    unsigned int* pixels = static_cast<unsigned int*>(malloc(raw.size));

    raw.width = pixelWidth;
    raw.height = pixelHeight;
    raw.data = pixels;

    if (!fishnet.paintMapClouds(raw)) {
        paintRect(pixels, pixelWidth,
            0, 0,
            0xFFFFFFFF,
            pixelWidth, pixelHeight
        );
    }

    std::lock_guard<std::mutex> lock(mapCloudListMutex);

    for (auto element = mapCloudList.begin(); element != mapCloudList.end(); element++) {
        MapCloud &mc = **element;

        //int ix = mc.lat - centerLat + width/2 - 1;
        //int iy = mc.lon - centerLon + height/2 - 1;

        int ix = mc.lat - minX;
        int iy = mc.lon - minY;

        if (ix < 0 || ix >= width || iy < 0 || iy >= height) break;

        unsigned char r = 0;
        unsigned char g = 0;
        unsigned char b = 0;

        if (mc.getRefNum() > 0) {
            int refs = mc.getRefNum();
            int refsMax = 6;
            b = 0xFF;
            g = refs > refsMax ? 0xFF : (refs * 0xFF / refsMax);
        }
        else {
            r = g = b = std::min(0xDD, (int)(mapCloudAccess - mc.getAccessTime() - 1));
        }

        unsigned int color = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
        color |= 0xFF000000;

        paintMapCloud(raw, mc.lat, mc.lon, color, tileSize);
    }
}

static void renderMapClouds(MapImage &img)
{
    MapImage raw;
    renderMapCloudsRaw(raw);
    encodeImage(&img, static_cast<unsigned int*>(raw.data), raw.width, raw.height);
}


MapCloudRef acquireMapCloud(int lat, int lon)
{

    int index = 0;
    for (auto element = mapCloudList.begin(); element != mapCloudList.end(); element++) {
        if ((*element)->lat == lat && (*element)->lon == lon) {
            //plog("map cloud %3d %3d found", lat, lon);
            return MapCloudRef(*element);
        }
        index++;
    }

    // Not found, create a new one
    std::string name = fmt::format(nameFormat, lat, lon);
    std::string block = fishnet.getBlockFromName(name);

    if (block != fishnet.blockNotAvailable) {
        std::string gkotPath = fmt::format(gkotFullFormat, block, name);
        std::string dof84Path = fmt::format(dof84FullFormat, block, name);
        std::string bdmrPath = fmt::format(bdmrFullFormat, block, name);

        normalizeSlashes(const_cast<char*>(gkotPath.c_str()));
        normalizeSlashes(const_cast<char*>(dof84Path.c_str()));
        normalizeSlashes(const_cast<char*>(bdmrPath.c_str()));

        MapCloud *mc = new MapCloud(lat, lon, gkotPath, dof84Path, bdmrPath);
        mapCloudList.push_front(mc);
        mc->acquire();
        trimMapCloudList();
        mc->release();
        //plog("map cloud %3d %3d created", lat, lon);
        return MapCloudRef(mc);
    }

    //plog("map cloud %3d %3d not found", lat, lon);
    return MapCloudRef(nullptr);
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

static void getBlockFromCoords(Vec reference, Vec coords, int &bx, int &by, int &bz)
{
    Vec diff = coords - reference;
    bx = static_cast<int>(floor(diff.x()));
    by = static_cast<int>(floor(diff.z()));
    bz = static_cast<int>(floor(-diff.y()));
}

static void getBlockFromCoords(Vec reference, Point p, int &bx, int &by, int &bz)
{
    Vec coords;
    coords << p.x, p.y, p.z;
    getBlockFromCoords(reference, coords, bx, by, bz);
}

static void getCoordsFromBlock(const Vec &reference, const int bx, const int by, const int bz, Vec &coords)
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

static void setCornersFromRefs(const MapCloudRef(&cls)[4], MapCloud* (&corners)[4]) {
    corners[0] = cls[0].cloud;
    corners[1] = cls[1].cloud;
    corners[2] = cls[2].cloud;
    corners[3] = cls[3].cloud;
}

static int getCornerMapClouds(MapCloudRef(&cornerClouds)[4], const Vec min, const Vec max) {
    const int cornerNum = 4;
    const pcln corners[cornerNum][2] = {
        { min.x(), min.y() },{ max.x(), min.y() },
        { min.x(), max.y() },{ max.x(), max.y() }
    };

    int latlonCount = 0;
    int latlon[cornerNum][3];

    for (int i = 0; i < cornerNum; i++) {

        int lat = static_cast<int>(floor(round(corners[i][0]) / mapTileWidth));
        int lon = static_cast<int>(floor(round(corners[i][1]) / mapTileHeight));

        //plog("%f %d %f %d", corners[i][0], lat, corners[i][1], lon);

        //lat = std::max(std::min(lat, lat_max), lat_min);
        //lon = std::max(std::min(lon, lon_max), lon_min);

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
        //plog("map cloud %3d %3d corner %d", lat, lon, i);
        cornerClouds[index] = acquireMapCloud(lat, lon);
        //MapCloud *mc = getMapCloud(lat, lon);
        //cornerClouds[index] = mc;
    }

    return latlonCount;
}


static std::mutex boxvisMutex;

static void getBounds(const Vec &origin,
    const int x, const int y, const int z,
    const int sx, const int sy, const int sz,
    Vec &bounds_tl, Vec &bounds_br,
    Vec &bounds_min, Vec &bounds_max
) {
    getCoordsFromBlock(origin, x, y, z, bounds_tl);
    getCoordsFromBlock(origin, x + sx, y + sy, z + sz, bounds_br);
    bounds_min = bounds_tl.cwiseMin(bounds_br);
    bounds_max = bounds_tl.cwiseMax(bounds_br);
}

// Position and size of the requested box (all block coordinates)
// Returns mutex locked box (unlock when done)
BoxResult& getBox(const uint32_t worldHash, Vec origin, const long x, long y, const long z, const long sx, long sy, const long sz, const bool debug = false) {

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

    // Required box height
    int rsy = sy;
    // Required box difference offset
    int ry = -1;

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
        br.origin == origin &&
        br.x == x && br.y == y && br.z == z &&
        br.sx == sx && br.sy == sy && br.sz == sz) {
        return br;
    }

    dtimer("box generation");

    //         //
    // Process //
    //         //

    // Not found in cache, update cached params
    if (!br.valid) ++boxesCached;
    br.valid = false;
    br.origin = origin;
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

    std::vector<unsigned int> blocks(sxyz, 0);
    std::vector<unsigned int> columns(sxz, 0);
    std::vector<unsigned int> groundCols(sxz, 0);

    size_t pointsUsed = 0;
    int minHeight = sy;
    int maxHeight = -1;

    // x -> z
    // y -> x
    // z -> y

    //static const int lat_min = MININT, lat_max = MAXINT, lon_min = MININT, lon_max = MAXINT;
    //static const int lat_min = 457, lat_max = 467, lon_min = 96, lon_max = 106;
    //static const int lat_min = 462, lat_max = 462, lon_min = 101, lon_max = 101;

    {
        //             //
        // Auto-height //
        //             //

        if (origin.z() == MAXLONG) {
            dtimer("height read");
            MapCloudRef mcl = acquireMapCloud((int)(origin.x() / mapTileWidth), (int)(origin.y() / mapTileHeight));
            MapCloud *mc = mcl.cloud;
            if (mc) {
                int mx, my;
                mc->getMapCoords(origin.x(), origin.y(), &mx, &my);
                origin.z() = mc->getPointHeight(mx, my) - sy / 2;
                if (isnan(origin.z())) origin.z() = 0;
            }
            else {
                origin.z() = 0;
            }
        }
    }

    bool shrink = true;
    bool shrunk = false;


    Vec bounds_tl;
    Vec bounds_br;
    Vec bounds_min;
    Vec bounds_max;

    // Get bounds
    getBounds(origin, x, y, z, sx, sy, sz, bounds_tl, bounds_br, bounds_min, bounds_max);

    int seaDummy, seaY;
    getBlockFromCoords(bounds_tl, Vec(0, 0, seaThreshold), seaDummy, seaY, seaDummy);

    PointCloud all(50);
    PointCloud ground(20);

    MapCloudRef cornerClouds[4];

    {
        //            //
        // Point load //
        //            //
        dtimer("point load");

        int count = getCornerMapClouds(cornerClouds, bounds_min, bounds_max);
        for (int i = 0; i < 4; i++) {
            MapCloud* mc = cornerClouds[i].cloud;
            if (!mc) continue;
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

            if (p.classification == Classification::OVERLAP) continue;

            int bx, by, bz;
            getBlockFromCoords(bounds_tl, p, bx, by, bz);

            if (bx < 0 || bx >= sx ||
                by < 0 || by >= sy ||
                bz < 0 || bz >= sz) continue;

            int index = getBlockIndex(bx, by, bz, sx, sxz);

            if (p.classification <= Classification::UNASSIGNED &&
                blocks[index] > Classification::UNASSIGNED) continue;

            blocks[index] = p.classification;

            if (by < minHeight) minHeight = by;
            if (by > maxHeight) maxHeight = by;
            
            //printf("%d ", p.classification);

            //br.exportWrite(bx + 0.5, bz + 0.5, by + 0.5, -10);

            pointsUsed++;
        }
    }

    {
        //                      //
        // Empty box into water //
        //                      //
        dtimer("box water");

        if (pointsUsed == 0) {
            int cornerNum = 4;
            Vec query_block_center;
            ClassificationQuery cq;
            setCornersFromRefs(cornerClouds, cq.corners);
            cq.lidar = Classification::NONE;

            int waters = 0;
            int others = 0;

            pcln heightSum = 0;
            int heightNum = 0;

            for (int iz = 0; iz < sz; iz++) {
                for (int ix = 0; ix < sx; ix++) {
                    getCoordsFromBlock(bounds_tl, ix, 0, iz, query_block_center);
                    cq.x = query_block_center.x();
                    cq.y = query_block_center.y();
                    MapCloud* mc;
                    int mapX, mapY;
                    Classification classification = MapCloud::getSpecializedClassification(cq, &mc, &mapX, &mapY);
                    if (mc) {
                        pcln height = mc->getPointHeight(mapX, mapY);

                        /* Per-column heights
                        int dummy;
                        int blockHeight;
                        query_block_center.z() = height;
                        getBlockFromCoords(bounds_tl, query_block_center, dummy, blockHeight, dummy);
                        extendColumn(ix, blockHeight, iz, sx, columns.data(), &minHeight, &maxHeight);
                        */

                        heightSum += height;
                        heightNum++;
                    }
                    if (classification == Classification::WATER) {
                        waters++;
                    } else {
                        others++;
                    }
                }
            }

            if (waters > others) {
                /* Per-column heights
                minHeight = maxHeight - (int)waterMaxDepth - 1;
                for (int iy = 0; iy <= sy-1; iy++) {
                    for (int iz = 0; iz < sz; iz++) {
                        for (int ix = 0; ix < sx; ix++) {
                            Classification classification;
                            if (iy == minHeight) {
                                classification = Classification::VEGETATION_LOW;
                            } else {
                                int col = columns[getColumnIndex(ix, iz, sx)];
                                classification = iy <= col ? Classification::BUILDING_ROOF_TILED_ORANGE : Classification::NONE;
                            }
                            int index = getBlockIndex(ix, iy, iz, sx, sxz);
                            blocks[index] = iy <= col ? BLOCK_SAND : BLOCK_AIR;//classificationToBlock(classification);
                        }
                    }
                }

                minHeight = 0;
                maxHeight = sy - 1;
                */

                //*
                pcln heightAvg = heightSum / heightNum;
                int dummy;
                int waterHeight;
                query_block_center.z() = std::round(heightAvg);
                getBlockFromCoords(bounds_tl, query_block_center, dummy, waterHeight, dummy);

                maxHeight = waterHeight;
                minHeight = maxHeight - (int)waterMaxDepth - 1;

                maxHeight = std::min((int)sy - 1, std::max(0, maxHeight));
                minHeight = std::min((int)sy - 1, std::max(0, minHeight));

                int height = std::min(maxHeight, std::max(minHeight, waterHeight));
                
                for (int iy = minHeight; iy <= maxHeight; iy++) {
                    int bid = iy == minHeight ?
                        classificationToBlock(Classification::GROUND) :
                        classificationToBlock(Classification::WATER);

                    for (int iz = 0; iz < sz; iz++) {
                        for (int ix = 0; ix < sx; ix++) {
                            int index = getBlockIndex(ix, iy, iz, sx, sxz);
                            blocks[index] = bid;
                            extendColumn(ix, iy, iz, sx, columns.data(), &minHeight, &maxHeight);
                        }
                    }

                }
                //*/
            }
        }
    }

    {
        //               //
        // Box shrinking //
        //               //
        dtimer("box shrink");

        if (shrink && maxHeight > -1) {
            // Add some buffer on top and bottom
            const int padTop = 1;
            const int padBottom = 1 + (int)waterMaxDepth;
            rsy = maxHeight - minHeight + padTop + padBottom;
            ry = minHeight - padBottom;
            if (ry < 0) {
                rsy += ry;
                ry = 0;
            }

            // Shrink box to required height
            if (rsy < sy) {
                int reqFrom = getBlockIndex(0, ry, 0, sx, sxz);
                int reqTo = getBlockIndex(0, ry + rsy, 0, sx, sxz);
                blocks.erase(blocks.begin() + reqTo, blocks.end());
                blocks.erase(blocks.begin(), blocks.begin() + reqFrom);
                sy = rsy;
                y += ry;
                sxyz = sxz*sy;

                getBounds(origin, x, y, z, sx, sy, sz, bounds_tl, bounds_br, bounds_min, bounds_max);
                getBlockFromCoords(bounds_tl, Vec(0, 0, seaThreshold), seaDummy, seaY, seaDummy);

                shrunk = true;
            }
        }
    }

    if (pointsUsed == 0 && maxHeight == -1) {
        blocks.resize(0);
    }

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
    if (pointsUsed > 0) {

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
                    if (cv == Classification::GROUND || cv == Classification::WATER) {
                        extendColumn(bx, by, bz, sx, groundCols.data());
                    }
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

            for (int i = 0; i < sxyz; i++) {
                unsigned int &cv = cblocks[i];
                if (cv == Classification::BUILDING) {

                    int bx, by, bz;

                    getIndexBlock(i, bx, by, bz, sx, sz);

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
        //*/

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

                    by = by < 0 ? 0 :
                         by < seaY ? seaY :
                         by >= sy ? sy - 1 :
                         by;

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
                                unsigned int &cv = cblocks[index];

                                if (iy <= seaY) {
                                    cv = Classification::WATER;
                                } else if (cv <= Classification::UNASSIGNED) {
                                    cv = Classification::GROUND;
                                }
                                
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
                    setCornersFromRefs(cornerClouds, cq.corners);
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
                        size_t minIndex = -1;

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
                        pcln diffRatio = points.size() > 0 ? ((pcln)targetPoints - sourcePoints) / points.size() : 0;

                        bool changed = false;
                        if (diffRatio > thresholdRatio) {
                            if (targetClosest) {
                                if (!isinf(minDist)) {
                                    cv = all.getPoint(minIndex).classification;
                                    changed = true;
                                }
                            } else {
                                cv = target;
                                changed = true;
                            }
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
                        int step = static_cast<int>(by) < surfy ? 1 : -1;
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

                        int maxDist = (int)waterMaxDepth;

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


    }

    //br.jsonClose();
    br.exportClose();

    // Fix height so it's above the highest block
    maxHeight++;

    // For an empty box, report as such
    if (maxHeight == 0) maxHeight = -2;

    //plog("num %lld ", num);

    {
        dtimer("serialization");

        const int size_int = 4;

        //*
        amf::Serializer serializer;
        serializer << amf::AmfVector<unsigned int>(blocks);
        serializer << amf::AmfVector<unsigned int>(columns);

        amf::v8 arrays = serializer.data();

        size_t dataSize = 6 * size_int + arrays.size() + 1 * size_int;
        //*/

        /* Compress into length + ints
        size_t dataSize =
            6 * size_int + 
            (1 + blocks.size() + 1 + columns.size())*size_int + 
            1 * size_int;
        //*/

        void *data = br.getBuffer(dataSize);
        vassert(br.dataSize == dataSize, "%d %d", (int)br.dataSize, (int)dataSize);
        amf::u8 *p = static_cast<amf::u8*>(data);

        writeUInt(p, worldHash); p += size_int;
        writeInt(p, ry); p += size_int;
        writeInt(p, rsy); p += size_int;
        writeInt(p, bx); p += size_int;
        writeInt(p, by); p += size_int;
        writeInt(p, bz); p += size_int;
        memcpy(p, arrays.data(), arrays.size()); p += arrays.size();
        // Compress into length + ints
        //p = writeUIntVector(p, blocks);
        //p = writeUIntVector(p, columns);
        writeInt(p, maxHeight); p += size_int;

        bool compress = maxHeight >= 0;

        if (!compress) br.removeCompressed();
        br.compression = BoxCompression::None;
        if (compress) br.compress();
        br.removeRedundant();
        br.valid = true;
    }

    ++boxesCreated;

    return br;
}

void GKOTHandleBox(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
    Vec origin;
    uint32_t worldHash;
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

    worldHash = getParamUInt(qs, ql, "worldHash");

    origin = {
        (double)getParamLong(qs, ql, "tmx"),
        (double)getParamLong(qs, ql, "tmy"),
        (double)getParamLong(qs, ql, "tmz", MAXLONG),
    };

    if (origin[0] == 0) origin[0] = default_origin[0];
    if (origin[1] == 0) origin[1] = default_origin[1];
    //if (origin[2] == 0) origin[2] = default_origin[2];

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

    debugPrint("origin: %f %f %f\n", origin[0], origin[1], origin[2]);
    debugPrint("x: %d\n", x);
    debugPrint("y: %d\n", y);
    debugPrint("z: %d\n", z);
    debugPrint("sx / w: %d\n", sx);
    debugPrint("sy    : %d\n", sy);
    debugPrint("sz / h: %d\n", sz);

    BoxResult &br = getBox(worldHash, origin, x, y, z, sx, sy, sz, debug);

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
}


void GKOTHandleOriginInfo(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
    const char *qs = info->query_string;
    if (!qs) {
        mg_send_http_error(conn, 400, "Missing query parameters");
        return;
    }

    size_t ql = strlen(info->query_string);

    int sy = 0;
    
    Vec origin = {
        (double)getParamLong(qs, ql, "tmx"),
        (double)getParamLong(qs, ql, "tmy"),
        NAN
    };

    const pcln offsetSpread = 1000;
    const int offsetNum = 5;
    const int randNum = 200;
    const int offsets[offsetNum][2] = {
        {  0,  0 },
        { -1, -1 }, {  1, -1},
        { -1,  1 }, { -1,  1}
    };

    pcln zmin = std::numeric_limits<pcln>::max();
    pcln zmax = std::numeric_limits<pcln>::min();

    std::default_random_engine generator;
    std::uniform_real_distribution<pcln> distribution(-offsetSpread, offsetSpread);

    auto random = std::bind(distribution, generator);

    for (int oi = 0; oi < offsetNum + randNum; oi++) {
        pcln ox, oy;
        if (oi < offsetNum) {
            const int* offset = offsets[oi];
            ox = offset[0] * offsetSpread;
            oy = offset[1] * offsetSpread;
        } else {
            ox = random();
            oy = random();
        }
        pcln x = origin.x() + ox;
        pcln y = origin.y() + oy;
        MapCloudRef mcl = acquireMapCloud((int)(x / mapTileWidth), (int)(y / mapTileHeight));
        MapCloud *mc = mcl.cloud;
        if (mc) {
            int mx, my;
            mc->getMapCoords(x, y, &mx, &my);
            pcln height = mc->getPointHeight(mx, my);
            if (!isnan(height)) {
                zmin = std::min(zmin, height);
                zmax = std::max(zmax, height);
                //printf("%3d %4.2f %4.2f\n", oi, zmin, zmax);
                if (ox == 0 && oy == 0) origin.z() = height;
            }
        }
    }

    if (isnan(origin.z())) origin.z() = -1;
    if (isnan(zmin)) zmin = 100000;
    if (isnan(zmax)) zmax = -100000;

    ujson::value result = ujson::object{
        { "tmx", origin.x() },
        { "tmy", origin.y() },
        { "tmz", origin.z() },
        { "zmin", zmin },
        { "zmax", zmax },
        { "zrange", zmax - zmin }
    };

    mg_printf(conn, "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n");

    std::string resultString = ujson::to_string(result);

    mg_printf(conn, "%s", resultString.c_str());
    
}


enum TileType {
    TYPE_INVALID,
    TYPE_LIDAR,
    TYPE_RASTER,
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
        token_type == "raster" ? TYPE_RASTER :
        TYPE_INVALID;

    if (type == TYPE_INVALID) { mg_send_http_error(conn, 400, "Unsupported tile type: %s", token_type.c_str()); return; }
    
    Vec req_origin;
    long sx, sy = 0, sz, x, y = 0, z;

    // Parse the rest of the name based on the type
    switch (type)
    {
        case TYPE_RASTER: {
            req_origin[0] = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
            req_origin[1] = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
            req_origin[2] = 0;

            sx = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
            sz = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
            x = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
            z = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
        }; break;

        default: {
            sx = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
            sy = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
            sz = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
            x = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
            y = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
            z = strtol(request_tokens[token_index++].c_str(), nullptr, 10);
        }; break;
    }

    int sxyz = sx*sy*sz;
    int sxz = sx*sz;

    //*
    std::vector<unsigned int>* blocks = nullptr;
    std::vector<unsigned int>* columns = nullptr;

    amf::AmfVector<unsigned int> amfblocks;
    amf::AmfVector<unsigned int> amfcolumns;
    //*/

    /*
    std::vector<unsigned int> blocks;
    std::vector<unsigned int> columns;
    */

    MapCloudRef cornerClouds[4];

    Vec bounds_tl;
    Vec bounds_br;
    Vec bounds_min;
    Vec bounds_max;

    // Load source data
    switch (type)
    {
    case TYPE_RASTER: {
        getCoordsFromBlock(req_origin, x, y, z, bounds_tl);
        getCoordsFromBlock(req_origin, x + sx, y + sy, z + sz, bounds_br);
        bounds_min = bounds_tl.cwiseMin(bounds_br);
        bounds_max = bounds_tl.cwiseMax(bounds_br);

        int count = getCornerMapClouds(cornerClouds, bounds_min, bounds_max);

        /*
        plog("%d map clouds", count);
        for (int i = 0; i < 4; i++) {
            MapCloud* mc = cornerClouds[i];
            if (!mc) continue;
            plog("%d: lat %d lon %d", i, mc->lat, mc->lon);
        }
        */

        }; break;

    default: {

        dtimer("deserialization");

        BoxResult &br = getBox(0, default_origin, x, y, z, sx, sy, sz, true);

        br.decompress();

        const int size_int = 4;
        uint32_t worldHash;
        int bx, by, bz, maxHeight;
        int ry, rsy;

        // This is stupid, but I don't know how else to make the stupid
        // iterators work with the deserializer below
        amf::v8 data(static_cast<amf::u8*>(br.data), static_cast<amf::u8*>(br.data) + br.dataSize);

        auto it = data.cbegin();
        auto end = it + br.dataSize;

        size_t datasize = br.dataSize;

        worldHash = readUInt(&*it); std::advance(it, size_int);
        ry = readInt(&*it); std::advance(it, size_int);
        rsy = readInt(&*it); std::advance(it, size_int);
        bx = readInt(&*it); std::advance(it, size_int);
        by = readInt(&*it); std::advance(it, size_int);
        bz = readInt(&*it); std::advance(it, size_int);

        if (ry != -1) {
            sy = rsy;
            sxyz = sxz*sy;
        }

        //auto blocks_end = it;
        //std::advance(blocks_end, sxyz*size_int);
        //size_t size = std::distance(it, blocks_end);
        //size_t vsize = br.data->size();
        //size_t toend = std::distance(it, br.data->cend());
        //size_t tsize = static_cast<size_t>(blocks_end - it);
        
        //*
        amf::Deserializer deserializer;
        amfblocks = deserializer.deserialize(it, end).as<amf::AmfVector<unsigned int>>();
        amfcolumns = deserializer.deserialize(it, end).as<amf::AmfVector<unsigned int>>();
        blocks = &amfblocks.values;
        columns = &amfcolumns.values;
        //*/

        //readUIntVector(&*it, blocks); std::advance(it, (1 + blocks.size())*size_int);
        //readUIntVector(&*it, columns); std::advance(it, (1 + columns.size())*size_int);

        maxHeight = readInt(&*it); std::advance(it, size_int);

        br.removeRedundant();

        br.mutex.unlock();

        }; break;
    }

    unsigned int *pixels;
    int width = sx;
    int height = sz;
    pixels = (unsigned int*)calloc(width*height, 4);

    vassert(pixels, "Unable to allocate memory for tile pixels");

    {
        dtimer("painting");

        MapCloud* corners[4];
        setCornersFromRefs(cornerClouds, corners);

        for (int i = 0; i < width*height; i++) {

            int classification;
            unsigned int rgb = 0xFF000000;

            switch (type)
            {
                case TYPE_RASTER: {
                    Vec coords;
                    int ox = i%width;
                    int oz = i/width;
                    getCoordsFromBlock(req_origin, x + ox, y, z + oz, coords);
                    rgb = MapCloud::getMapColor(corners, coords.x(), coords.y());
                    /*
                    rgb = -1;
                    int mx, my;
                    MapCloud* mc = MapCloud::fromCorners(cornerClouds, coords.x(), coords.y(), &mx, &my);
                    if (mc) rgb = mc->lon*mc->lat;
                    */
                    if (rgb == -1) rgb = 0x000000;
                }; break;

                default:
                    int h = columns->at(i);
                    int topBlockIndex = i + h*sxz;
                    classification = blocks->at(topBlockIndex);
            }

            /*
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
            */

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
    if (status == -1) plog("Unable to send dashboard boxes");
}

void GKOTHandleDashboardMapClouds(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
    dtimer("dash map clouds");
    MapImage img;
    renderMapClouds(img);
    sendLiveImageHeader(conn, img.size);
    int status = mg_write(conn, img.data, img.size);
    if (status == -1) plog("Unable to send dashboard map clouds");
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
    //lasreadopener.set_file_name(fmt::format(gkotFullFormat, "b_35", "462_101").c_str());
    lasreadopener.set_file_name(fmt::format(gkotFullFormat, "b_21", "387_44").c_str());
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
        int index = by + (mw - 1 - bx)*mh;
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

    MapCloudRef mcl = acquireMapCloud(462, 101);
    MapCloud *mc = mcl.cloud;

    int mw = mapTileWidth;
    int mh = mapTileHeight;

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


void GKOTHandleDebugHeight(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{

    mg_printf(conn, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
    mg_printf(conn, "<html><body>");
    mg_printf(conn, "Hello heights!<br><pre>");

    int mw = bdmrWidth;
    int mh = bdmrHeight;

    MapCloudRef mcl = acquireMapCloud(388, 440);
    MapCloud *mc = mcl.cloud;

    char *pixels = (char*)malloc(mw*mh * 3);
    assert(pixels);

    if (mc != nullptr) {
        pcln origin = mc->getPointHeight(500, 500);

        int mapX, mapY;
        mc->getMapCoords(460666.07, 101851.476, &mapX, &mapY);
        mg_printf(conn, "tivolski grad: %d %d %f<br/>", mapX, mapY, mc->getPointHeight(mapX, mapY));

        for (int i = 0; i < mw*mh; i++) {
            int mx = i%mw;
            int my = i / mw;
            int pi = i * 3;
            pcln v = mc->getPointHeight(mx, my);
            int r = max(0, min(0xFF, 0x7F + (int)(v - origin)));
            int g = r;
            int b = r;
            pixels[pi + 0] = r;
            pixels[pi + 1] = g;
            pixels[pi + 2] = b;
        }
    }

    MapImage img;
    stbi_write_png_to_func(&writeToBuffer, &img, mw, mh, 3, pixels, 0);

    free(pixels);

    std::string pngDataURI;

    base64::encoder encoder;
    std::istringstream pngIn = std::istringstream(std::string(static_cast<const char*>(img.data), img.size));
    std::ostringstream pngOut;
    encoder.encode(pngIn, pngOut);
    pngDataURI = "data:image/png;base64," + pngOut.str();

    mg_printf(conn, "</pre><img width=\"%d\" height=\"%d\" alt=\"map\" src=\"%s\" />", mw, mh, pngDataURI.c_str());


    mg_printf(conn, "</body></html>\n");
}



void DOF84HandleDebug(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
    mg_printf(conn, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
    mg_printf(conn, "<html><body>");
    mg_printf(conn, "Hello!<pre>");
    
    MapCloudRef mcl = acquireMapCloud(462, 101);
    MapCloud *mc = mcl.cloud;

    int mw = mapTileWidth;
    int mh = mapTileHeight;

    PointCloud all(50);
    PointCloud ground(20);

    mc->load(&all, &ground);

    Vec origin;
    origin << 462000, 101000, 0;

    unsigned char *map = static_cast<unsigned char*>(mc->getMap().data);
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
    const mg_request_info *info = mg_get_request_info(conn);

    if (strcmp(info->request_uri, "/gkot/box") == 0) {
        GKOTHandleBox(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/gkot/origin.json") == 0) {
        GKOTHandleOriginInfo(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/dashboard/") == 0) {
        mg_send_file(conn, (webPath + "/dashboard/dashboard.html").c_str());
    } else if (strcmp(info->request_uri, "/dashboard/boxes.png") == 0) {
        GKOTHandleDashboardBoxes(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/dashboard/mapClouds.png") == 0) {
        GKOTHandleDashboardMapClouds(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/dashboard/stats.json") == 0) {
        GKOTHandleDashboardStats(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/gkot/debug") == 0) {
        GKOTHandleDebug(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/gkot/debugHeight") == 0) {
        GKOTHandleDebugHeight(conn, cbdata, info);
    /*
    } else if (strcmp(info->request_uri, "/gkot/debugBoxes") == 0) {
        GKOTHandleDebugBoxes(conn, cbdata, info);
    */
    } else if (startsWith(info->request_uri, "/debug/tile_") && endsWith(info->request_uri, ".png")) {
        GKOTHandleTile(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/debug/") == 0) {
        mg_send_file(conn, (webPath + "/debug/debug.html").c_str());
    } else if (strcmp(info->request_uri, "/dof84") == 0) {
        DOF84HandleDebug(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/") == 0) {
        mg_send_file(conn, (webPath + "/index.html").c_str());
    } else if (startsWith(info->request_uri, "/")) {
        if (strstr(info->request_uri, "..") != NULL) return 0;
        std::string path = webPath + "/" + info->request_uri;
        mg_send_file(conn, path.c_str());
    } else {
        return 0;
    }

    ++requestsServed;

    plog("%s %s?%s", info->request_method, info->request_uri, info->query_string);
    
    return 1;
}

enum  OptionIndex { UNKNOWN, HELP, PORT, WWW, LIDAR, MAP, BDMR, FISHNET, ORIGIN, CACHE, MAP_MEMORY_LIMIT };
const option::Descriptor usage[] =
{
    { UNKNOWN, 0, "",  "",      option::Arg::None,     "USAGE: voxelserver [options] [root-path-to-gis-data]\n\n"
    "Options:" },
    { HELP,    0, "h", "help",    option::Arg::None,     "  --help, -h  \tPrint usage and exit." },
    { PORT,    0, "p", "port",    option::Arg::Optional, "  --port, -p  \tServer listening port number." },
    { LIDAR,   0, "l", "lidar",   option::Arg::Optional, "  --lidar, -l  \tPath to the LIDAR sections directory in GKOT format." },
    { MAP,     0, "m", "map",     option::Arg::Optional, "  --map, -m  \tPath to the map sections directory in DOF84 format." },
    { BDMR,    0, "r", "bdmr",    option::Arg::Optional, "  --bdmr, -r  \tPath to the binary digital relief model sections directory." },
    { FISHNET, 0, "d", "fishnet", option::Arg::Optional, "  --fishnet, -d  \tPath to the fishnet database of sections." },
    { WWW,     0, "w", "www",     option::Arg::Optional, "  --www, -w  \tPath to the directory containing web files." },
    { ORIGIN,  0, "o", "origin",  option::Arg::Optional, "  --origin, -o  \tD96/TM coordinates of the box origin." },
    { CACHE,    0, "c", "cache",  option::Arg::Optional, "  --cache, -c  \tCache hash size power, default 14." },
    { MAP_MEMORY_LIMIT, 0, "t", "map-memory", option::Arg::Optional, "  --map-memory, -t  \tMap memory limit in megabytes." },
    { UNKNOWN, 0, "",  "",        option::Arg::None,     "\n  Paths can be absolute or relative to the root path." },
    { UNKNOWN, 0, "",  "",        option::Arg::None,     "\nExamples:\n"
                                                         "  voxelserver\n"
                                                         "  voxelserver --port=8989 W:/gis/arso/\n"
                                                         "  voxelserver -o\"461777 101414 200\" W:/gis/arso/\n"
    },
    { 0,0,0,0,0,0 }
};

static std::string getPathOption(const option::Option* options, const std::string& root, OptionIndex index, const std::string& defaultPath)
{
    std::string path = options[index] ? options[index].arg : defaultPath;
    bool abs = path[0] == '/' || path[0] == '\\' || path[1] == ':';
    if (!abs) path = root + "/" + path;
    return path;
}

static pcln parseCoord(const std::string& coord)
{
    std::istringstream in(coord);
    in.imbue(locale("sl-SI"));
    pcln c;
    in >> c;
    return c;
}

static Vec getCoordsOption(const option::Option* options, OptionIndex index, Vec defaultCoords)
{
    const option::Option* opt = options[index];
    if (!opt) return defaultCoords;
    std::string coordStr = opt->arg;
    std::vector<std::string> spl = split(coordStr, ' ');
    if (spl.size() != 3) {
        plog("Three coordinates separated by space required: %s", coordStr.c_str());
        return defaultCoords;
    }
    return Vec(parseCoord(spl[0]), parseCoord(spl[1]), parseCoord(spl[2]));
}

int main(int argc, char const* argv[])
{
    plog("");
    plog("--- voxelserver ---");
    plog("");

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
    }

    std::string port, path, gkotAbsPath, dof84AbsPath, bdmrAbsPath;
    int hashPower;

    argc -= (argc>0); argv += (argc>0); // skip program name argv[0] if present
    option::Stats  stats(usage, argc, argv);

    std::vector<option::Option> options{ stats.options_max };
    std::vector<option::Option> optionBuffers{ stats.buffer_max };

    option::Parser parse(usage, argc, argv, &options[0], &optionBuffers[0]);

    if (parse.error()) return EXIT_FAILURE;

    if (options[HELP]) {
        option::printUsage(std::cout, usage);
        return EXIT_SUCCESS;
    }

    for (option::Option* opt = options[UNKNOWN]; opt; opt = opt->next())
        std::cout << " Unknown option: " << opt->name << std::endl;
    if (options[UNKNOWN]) std::cout << std::endl;

    path = parse.nonOptionsCount() > 0 ? parse.nonOption(0) : defaultPath;
    for (int i = 1; i < parse.nonOptionsCount(); ++i) {
        std::cout << "Unknown argument: " << parse.nonOption(i) << "\n";
    }

    // TODO: dodaj moznost spremembe bufferja in default pozicije

    port = options[PORT] ? options[PORT].arg : defaultPort;
    gkotAbsPath = getPathOption(&options[0], path, LIDAR, gkotRel);
    dof84AbsPath = getPathOption(&options[0], path, MAP, dof84Rel);
    bdmrAbsPath = getPathOption(&options[0], path, BDMR, bdmrRel);
    fishnetPath = getPathOption(&options[0], path, FISHNET, fishnetRel);
    webPath = getPathOption(&options[0], path, WWW, webRel);
    default_origin = getCoordsOption(&options[0], ORIGIN, default_origin);
    hashPower = options[CACHE] ? atoi(options[CACHE].arg) : defaultPower;
    vassert(hashPower > 0, "Hash power should be greater than zero: %d", hashPower);
    mapMemoryLimit = options[MAP_MEMORY_LIMIT] ? atoi(options[MAP_MEMORY_LIMIT].arg) : defaultMapMemoryLimit;
    vassert(mapMemoryLimit > 0, "Map memory limit should be greater than zero: %d", mapMemoryLimit);

    boxHash.resize(hashPower);

    gkotFullFormat = gkotAbsPath + "/" + gkotFormat;
    dof84FullFormat = dof84AbsPath + "/" + dof84Format;
    bdmrFullFormat = bdmrAbsPath + "/" + bdmrFormat;

    plog("Lidar (gkot) path: %s", gkotAbsPath.c_str());
    plog("Map (dof84) path: %s", dof84AbsPath.c_str());
    plog("BDMR path: %s", bdmrAbsPath.c_str());
    plog("Web files path: %s", webPath.c_str());
    plog("Fishnet database: %s", fishnetPath.c_str());
    plog("Default origin coordinates: %g, %g, %g", default_origin.x(), default_origin.y(), default_origin.z());
    plog("Box cache size: %d", boxHash.size);
    plog("Map memory limit: %d MB", mapMemoryLimit);
    
    bool dbLoaded = fishnet.load(fishnetPath.c_str());
    vassert(dbLoaded, "Unable to open fishnet database: %s", fishnetPath.c_str());

    const char *serverOptions[] = {
        "listening_ports", port.c_str(),
        "request_timeout_ms", "10000",
        //"error_log_file", "error.log",
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
    server = mg_start(&callbacks, 0, serverOptions);

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

        plog("");
        plog("Server up at %s://%s:%i/", proto, host, ports[n].port);
        plog("");
    }


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

