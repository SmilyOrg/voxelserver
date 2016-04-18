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

#include <sys/types.h>
#include <sys/stat.h>

#pragma warning(push)
#pragma warning(disable:4996)
#pragma warning(disable:4267)

#include "lasreader.hpp"
#include "laswriter.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#define BUFFERSIZE 4096
#include "b64/encode.h"


#include "civetweb/civetweb.h"

#pragma warning(disable:4244)

#include "amf-cpp/amf.hpp"
#include "amf-cpp/serializer.hpp"
#include "amf-cpp/types/amfdouble.hpp"
#include "amf-cpp/types/amfinteger.hpp"
#include "amf-cpp/types/amfvector.hpp"

#pragma warning(pop)

#include "nanoflann.hpp"

#include "ujson/ujson.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

#include <Eigen/Core>


#ifdef _WIN32
#include <direct.h>
#endif


#include "7zip/LzmaLib.h"
#include "7zip/LzmaEnc.h"


#define NO_CACHE \
    "Cache-Control: no-cache, no-store, must-revalidate\r\n" \
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

#define vassert(x, format, ...) if (!x) { printf(format, __VA_ARGS__); __debugbreak(); }

static std::atomic<int> plogLevel = 0;
#define plog(format, ...) printf("%*s" format "\n", plogLevel*2, " ", __VA_ARGS__)

struct PlogScope
{
    PlogScope() { plogLevel++; }
    ~PlogScope() { plogLevel--; }
};
#define plogScope() PlogScope plog_scope_struct

static const char *gkotPathFormat = "W:/gis/arso/laz/gkot/b_35/D96TM/TM_%d_%d.laz";
static const char *otrPathFormat = "W:/gis/arso/laz/otr/b_35/D96TM/TMR_%d_%d.laz";
static const char *nanoflannAllPathFormat = "W:/gis/arso/nanoflann/gkot/b_35/D96TM/TM_%d_%d.kdz";
static const char *nanoflannGroundPathFormat = "W:/gis/arso/nanoflann/otr/b_35/D96TM/TM_%d_%d.kdtree";

typedef double pcln;
typedef Eigen::Vector3d Vec;

enum Classification
{
    UNASSIGNED = 1,
    GROUND = 2,
    VEGETATION_LOW = 3,
    VEGETATION_MEDIUM = 4,
    VEGETATION_HIGH = 5,
    BUILDING = 6,
    LOW_POINT = 7,
    WATER = 9
};

enum BlockType
{
    BLOCK_AIR               = 0,
    BLOCK_DIRT              = 3,

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

    BLOCK_DOUBLE_STONE_SLAB = 43,
    
};




#if __cplusplus < 201103L && (!defined(_MSC_VER) || _MSC_VER < 1700)
#error Timer requires C++11
#else
#include <chrono>
class Timer {
    typedef std::chrono::high_resolution_clock clock;
    typedef std::chrono::nanoseconds ns;

    clock::time_point start;
    const char *name;
    bool printed = false;

public:
    Timer(const char *name)
    {
        this->name = name;
        tick();
    }
    ~Timer()
    {
        if (!printed) print();
    }
    void tick()
    {
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
        plog("%s %*s%ld%s", name, 20 - strlen(name), " ", c, unit);
    }
};
#endif

class RuntimeCounter;

class RuntimeCounters {

    typedef std::list<RuntimeCounter*> RCList;
    std::mutex mutex;

public:
    RCList list;

    void add(RuntimeCounter *c);

} runtimeCounters;


class RuntimeCounter {
    typedef unsigned long Count;

    std::atomic<Count> count;

public:
    const char *id;
    const char *name;

    RuntimeCounter(const char *id, const char *name) : id(id), name(name) {
        reset();
        runtimeCounters.add(this);
    }

    Count operator++() { return ++count; }
    Count operator+=(Count c) { count += c; return count; }

    Count reset() {
        Count c = count;
        count = 0;
        return c;
    }

};


void RuntimeCounters::add(RuntimeCounter *c) {
    std::lock_guard<std::mutex> lock(mutex);
    for (auto &iter : list) { assert(strcmp(iter->id, c->id) != 0); }
    list.push_back(c);
}


#define ADD_COUNTER(id, name) RuntimeCounter id ## (#id, name);

ujson::value to_json(RuntimeCounters const &rcs) {
    auto arr = ujson::array();
    for (auto &iter : rcs.list) {
        arr.push_back(
            ujson::object {
                { "id", iter->id },
                { "name", iter->name },
                { "value", static_cast<int>(iter->reset()) }
            }
        );
    }
    return arr;
}


ADD_COUNTER(boxesSent, "Boxes sent");
ADD_COUNTER(boxesCreated, "Boxes created");
ADD_COUNTER(pointsLoaded, "Points loaded");
ADD_COUNTER(requestsServed, "Requests served");



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

    MapImage() { data = NULL; }
    ~MapImage() {
        if (data) {
            free(data);
            data = NULL;
        }
    }
};

void writeToBuffer(void *context, void *data, int size) {
    MapImage *img = static_cast<MapImage*>(context);
    img->data = malloc(size);
    assert(img->data);
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
#if 0
#define debug(format, ...) plog(format, __VA_ARGS__)
#define dtimer(name) Timer timer(name)
#else
#define debug(format, ...) 
#define dtimer(name)  
#endif

void printArray(struct mg_connection *conn, unsigned char *data, size_t size, size_t cols = 32)
{
    mg_printf(conn, "\n\n        ");
    for (size_t i = 0; i < cols; i++) {
        mg_printf(conn, "%2d ", i);
    }
    mg_printf(conn, "\n\n");
    for (size_t i = 0; i < size; i++) {
        if (i % cols == 0) mg_printf(conn, "\n%6d  ", i);
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

void readInt(amf::u8 *p, int *v)
{
    *v = (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
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

    PointCloud(int maxLeaf) : tree(NULL) {
        tree = new KDTree(3, *this, KDTreeSingleIndexAdaptorParams(maxLeaf));
    }

    ~PointCloud() {
        if (tree) delete tree;
        tree = NULL;
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
    PointCloudIO() : reader(NULL) {}

    ~PointCloudIO()
    {
        close();
    }
    
    void open(const char *path)
    {
        LASreadOpener opener = LASreadOpener();
        opener.set_file_name(path);
        reader = opener.open();

        if (!reader) {
            plog("Unable to open %s", path);
            exit(1);
        }
    }

    void close()
    {
        if (reader) delete reader;
        reader = NULL;
    }

    void load(double min_x, double min_y, double max_x, double max_y, PointCloud *all, PointCloud *ground)
    {
        std::lock_guard<std::mutex> lock(mutex);

        assert(reader);
        vassert(reader->inside_none(), "Unable to reset LASreader bounds");
        vassert(reader->inside_rectangle(min_x, min_y, max_x, max_y), "Unable to set LASreader bounds");

        Point p;
        while (readPoint(p)) {
            all->addPoint(p);
            if (p.classification == Classification::GROUND) ground->addPoint(p);
        }
    }
};


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

    TreeIndex() : path(NULL), file(NULL) {}
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
        readInt(bytes, &n);
        *ptr = n;
        return true;
    }

    bool begin(const char *path) {
        cleanup();

        this->path = path;

        if (!writing) {
            // Reading
            file = fopen(path, "rb");
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

            fclose(file); file = NULL;
        }

        return true;
    }

    bool error() {
        cleanup();
        return false;
    }

    void cleanup() {
        if (file) { fclose(file); file = NULL; }
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
            
            if (propsSize != LZMA_PROPS_SIZE) { plog("Invalid props size: %d", propsSize); return error(); }
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

    PointSearch(const char* pclPath, const char* treePath, int maxLeaf) : cloud(PointCloud(pclPath)), tree(NULL) {
        this->maxLeaf = maxLeaf;
        this->treePath = treePath;
    }

    ~PointSearch() {
        deleteTree();
    }

    void deleteTree()
    {
        if (tree) delete tree;
        tree = NULL;
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

class MapCloud {
public:
    static const int READER_NUM = 6;
    const int lat;
    const int lon;

    bool readerFree[READER_NUM];
    PointCloudIO readers[READER_NUM];
    int readersFree = READER_NUM;

protected:
    const char *path;

    std::mutex mutex;
    std::condition_variable cond;
public:

    MapCloud(int lat, int lon, const char *path) : 
        lat(lat), lon(lon),
        path(path)
    {
        open();
    }

    ~MapCloud() {
        close();
    }

    void open() {
        close();
        for (int i = 0; i < READER_NUM; i++) {
            readers[i].open(path);
            readerFree[i] = true;
        }
    }

    void close() {
        for (int i = 0; i < READER_NUM; i++) {
            readerFree[i] = false;
            readers[i].close();
        }
    }

    void load(double min_x, double min_y, double max_x, double max_y, PointCloud *all, PointCloud *ground) {
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
            assert(readerIndex != -1);
            readersFree--;
        }

        reader->load(min_x, min_y, max_x, max_y, all, ground);

        {
            std::unique_lock<std::mutex> lock(mutex);
            readerFree[readerIndex] = true;
            reader = NULL;
            readersFree++;
            cond.notify_one();
        }
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
            hash = NULL;
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

    plog("");
    plog("Processing new map cloud at %d, %d", lat, lon);

    char allPath[1024], treeAllPath[1024],
         groundPath[1024], treeGroundPath[1024];

    sprintf_s(allPath, gkotPathFormat, lat, lon);
    sprintf_s(groundPath, otrPathFormat, lat, lon);
    sprintf_s(treeAllPath, nanoflannAllPathFormat, lat, lon);
    sprintf_s(treeGroundPath, nanoflannGroundPathFormat, lat, lon);

    MapCloud *mc = new MapCloud(lat, lon, allPath);
    mapCloudList.push_front(mc);

    //if (mc.all) delete mc.all;
    //if (mc.ground) delete mc.ground;

    plogScope();

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

    plog("Done");
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

    if (cv & (1 << 31)) return cv & ~(1 << 31);
    //if (cv & (1 << 31)) return 0;

    int classification = cv & 0xFF;
    int blocklight = (cv >> 8) & 0xF;

    switch (classification)
    {
    case 0:                                 bv = BLOCK_AIR; break;
    case Classification::UNASSIGNED:        bv = BLOCK_WOOL; break;
    case Classification::GROUND:            bv = BLOCK_DIRT; break;
    case Classification::VEGETATION_LOW:    bv = BLOCK_DOUBLE_STONE_SLAB; break;
    case Classification::VEGETATION_MEDIUM: bv = BLOCK_DOUBLE_STONE_SLAB; break;
    case Classification::BUILDING:          bv = BLOCK_WOOL_LIGHTGRAY; break;
    case Classification::VEGETATION_HIGH:   bv = BLOCK_WOOL_GREEN; break;
    case Classification::LOW_POINT:         bv = BLOCK_WOOL_BLACK; break;
    case Classification::WATER:             bv = BLOCK_WOOL_LIGHTBLUE; break;
    default:                                bv = BLOCK_WOOL_GRAY; break;
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

struct BoxResult {
    std::mutex mutex;
    
    std::atomic<long> access;

    bool valid;

    long x, y, z;
    long sx, sy, sz;

    amf::v8 *data;

    BoxResult() : data(NULL), access(0), valid(false) {};
    ~BoxResult() {
        if (data) delete data;
        data = NULL;
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
static std::atomic<long> boxHashAccess = boxHashAccessStart;
static SpatialHash<BoxResult> boxHash(10);

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

static inline int getBlockIndex(int bx, int by, int bz, int sx, int sxz)
{
    return bx + bz*sx + by*sxz;
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

template <typename num_t>
static inline void getBlockFromCoords(num_t *origin, Point &p, int &bx, int &by, int &bz)
{
    bx = static_cast<int>(p.y - origin[1]);
    by = static_cast<int>(p.z - origin[2]);
    bz = static_cast<int>(p.x - origin[0]);
}

static std::mutex boxvisMutex;

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

    //char status[128];

    const char *qs = info->query_string;
    size_t ql = strlen(info->query_string);

    x  = getParamLong(qs, ql, "x");
    y  = getParamLong(qs, ql, "y");
    z  = getParamLong(qs, ql, "z");
    sx = getParamLong(qs, ql, "sx");
    sy = getParamLong(qs, ql, "sy");
    sz = getParamLong(qs, ql, "sz");
    debug = getParamBool(qs, ql, "debug");

    assert(sx > 0);
    assert(sy > 0);
    assert(sz > 0);

    bool collision = false;

    int psx = (int)log2(sx);
    int psy = (int)log2(sy);
    int psz = (int)log2(sz);
    int msx = sx - 1;
    int msy = sy - 1;
    int msz = sz - 1;

    int bx = x >> psx;
    int by = y >> psy;
    int bz = z >> psz;

    debug("init");

    unsigned int boxHashCode;
    BoxResult &br = boxHash.at(bx, bz, boxHashCode);

    std::lock_guard<std::mutex> brLock(br.mutex);

    long access = ++boxHashAccess;
    br.access = access;
    
    //std::lock_guard<std::mutex> encodeLock(encodeMutex);

    /*
    MapImage img;
    renderBoxesUsed(img);

    char *filename;
    asprintf(&filename, "../vis/frames/boxvis-%06d.png", access-boxHashAccessStart);
    FILE *f = fopen(filename, "wb");
    fwrite(img.data, 1, img.size, f);
    fclose(f);
    //*/

    //printf(" box/%x ", boxHashCode);

    if (!debug && br.valid &&
        br.x  ==  x && br.y  == y  && br.z  == z &&
        br.sx == sx && br.sy == sy && br.sz == sz) {
        
        br.send(conn);
        boxesSent++;

        debug("cache");

        //sprintf_s<sizeof(status)>(status, "cached %08x", hashCode);
        //return "cached";
        return;
    }

    //BoxResult br;

    //sprintf_s<sizeof(status)>(status, "query %08x", hashCode);
    br.valid = false;
    br.x = x;
    br.y = y;
    br.z = z;
    br.sx = sx;
    br.sy = sy;
    br.sz = sz;
    
    debugPrint("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
    debugPrint("<html><body>");
    debugPrint("Hello!<br><pre>");

    debugPrint("x: %d\n", x);
    debugPrint("y: %d\n", y);
    debugPrint("z: %d\n", z);
    debugPrint("sx / w: %d\n", sx);
    debugPrint("sy    : %d\n", sy);
    debugPrint("sz / h: %d\n", sz);

    int sxyz = sx*sy*sz;
    int sxz = sx*sz;

    /*
    amf::AmfVector<unsigned int> amfblocks;
    std::vector<unsigned int> &blocks = amfblocks.values;
    //blocks.resize(sxyz);
    amfblocks.fixed = true;

    amf::AmfVector<unsigned int> amfcolumns;
    std::vector<unsigned int> &columns = amfcolumns.values;
    columns.resize(sxz);
    amfcolumns.fixed = true;
    */

    std::vector<unsigned int> blocks(sxyz);
    std::vector<unsigned int> columns(sxz);


    int count = 0;
    int minHeight = sy;
    int maxHeight = -1;

    // x -> z
    // y -> x
    // z -> y


    //static const int lat_min = 461, lat_max = 462, lon_min = 100, lon_max = 101;
    //static const int lat_min = 462, lat_max = 462, lon_min = 101, lon_max = 101;
    //static const int lat_min = 460, lat_max = 463, lon_min = 99, lon_max = 102;
    static const int lat_min = 457, lat_max = 467, lon_min = 96, lon_max = 106;

    static const int origin_z = 462000;
    static const int origin_x = 101000;
    static const int origin_y = 200;

    int abs_x = origin_x + x;
    int abs_y = origin_y + y;
    int abs_z = origin_z + z;

    int lat = abs_z / 1000;
    int lon = abs_x / 1000;

    lat = std::max(std::min(lat, lat_max), lat_min);
    lon = std::max(std::min(lon, lon_max), lon_min);

    /*
    unsigned int mapHashCode;
    MapCloud<double> &mc = mapCloudHash.at(lat, lon, mapHashCode);

    //std::lock_guard<std::recursive_mutex>(mc.mutex);

    ensureMapCloud<double>(mc, lat, lon, mapHashCode);
    */

    debug("mapcloud get");

    MapCloud &mc = *getMapCloud(lat, lon);
    
    //printf(" map ");

    /*
    Vec
        bounds_min,
        bounds_max,
        query_box_center;

    bounds_min = mc.all->cloud.getQuantizedPoint(abs_z, abs_x, abs_y);
    bounds_max = mc.all->cloud.getQuantizedPoint(abs_z + sz, abs_x + sx, abs_y + sy);
    query_box_center = (bounds_min + bounds_max) / 2;
    //*/

    /*
    Vec bmin, bmax;
    bmin << abs_z, abs_x, abs_y;
    bmax << abs_z + sz, abs_x + sx, abs_y + sy;

    mc.all->cloud.setBounds(bmin.x(), bmin.y(), bmax.x(), bmax.y());
    */

    Vec
        bounds_min,
        bounds_max,
        query_box_center;
    bounds_min << abs_z, abs_x, abs_y;
    bounds_max << abs_z + sz, abs_x + sx, abs_y + sy;
    query_box_center = (bounds_min + bounds_max) / 2;

    // 462006. 101005. 462016. 101015.00000000000

    /*
    //printf("seek %d\n", mc.all->cloud.reader->seek(0));
    printf("ins rec %d\n", mc.all->cloud.reader->inside_rectangle(462005., 101005, 462010., 101010.));
    //printf("seek %d\n", mc.all->cloud.reader->seek(0));
    printf("read point %d\n", mc.all->cloud.reader->read_point());
    printf("%f %f\n", mc.all->cloud.reader->point.get_x(), mc.all->cloud.reader->point.get_y());

    printf("ins none %d\n", mc.all->cloud.reader->inside_none());

    //printf("seek %d\n", mc.all->cloud.reader->seek(0));
    printf("ins rec %d\n", mc.all->cloud.reader->inside_rectangle(462030., 101030., 462040., 101040.));
    //printf("seek %d\n", mc.all->cloud.reader->seek(0));
    printf("read point %d\n", mc.all->cloud.reader->read_point());
    printf("%f %f\n", mc.all->cloud.reader->point.get_x(), mc.all->cloud.reader->point.get_y());
    //*/

    /*
    pcln bounds_min[3] = {
        abs_z,
        abs_x,
        abs_y
    };

    pcln bounds_max[3] = {
        bounds_min[0] + sz,
        bounds_min[1] + sx,
        bounds_min[2] + sy
    };

    pcln query_box_center[3] = {
        (bounds_min[0] + bounds_max[0])*0.5,
        (bounds_min[1] + bounds_max[1])*0.5,
        (bounds_min[2] + bounds_max[2])*0.5
    };
    //*/

    //ProgressPrinter pp;

    //FILE *dp = fopen("data.json", "w");
    //fprintf(dp, "{ \"points\": [\n");

    // YZX
    // uint element value
    // 0x000000FF	block id
    // 0x00000F00	block data
    // 0x0000F000	skylight
    // 0x000F0000	blocklight
    // 0x0FF00000	? height of column (stored at y = 0)

    //Timer timer;

    //timer.tick();

    /*
    const pcln radius = sy/2 + 1;
    std::vector<std::pair<size_t, pcln>> indices;
    RadiusResultSet<pcln, size_t> results(radius*radius, indices);
    mc.all->findRadius(query_box_center.data(), results);
    */
    
    //mc.all->tree->findNeighbors(resultSet, query_box_center, nanoflann::SearchParams(32, 0, false));

    //mapCloudHash.mutex.unlock();

    //mc.mutex.unlock();

    //size_t num = results.size();

    size_t num = 0;

    //debugPrint("%d query took %d ms\n", timer.tock().count());

    //debugPrint("%d points found\n", num);


    PointCloud all(50);
    PointCloud ground(20);

    {
        //            //
        // Point load //
        //            //
        dtimer("point load");
        mc.load(bounds_min.x(), bounds_min.y(), bounds_max.x(), bounds_max.y(), &all, &ground);
    }

    //for (size_t i = 0; i < num; i++) {
    {
        //              //
        // Quantization //
        //              //
        dtimer("quantization");
        size_t num = all.getPointNum();
        pointsLoaded += num;
        for (size_t i = 0; i < num; i++) {
            Point &p = all.getPoint(i);

            //std::pair<size_t, pcln> pair = results.m_indices_dists[i];
            //mc.all->cloud.getPoint(pair.first, p);
            //pcln dist = pair.second;

            //debugPrint("%6d  %6f  %6f  %6f  %d    %6f", i, p.x, p.y, p.z, p.classification, dist);

            if (p.x < bounds_min.x() || p.x >= bounds_max.x() ||
                p.y < bounds_min.y() || p.y >= bounds_max.y() ||
                p.z < bounds_min.z() || p.z >= bounds_max.z()) continue;

            int bx, by, bz;
            getBlockFromCoords(bounds_min.data(), p, bx, by, bz);

            if (by < 0 || by > sy) continue;

            int index = getBlockIndex(bx, by, bz, sx, sxz);

            blocks[index] = p.classification;

            if (by < minHeight) minHeight = by;
            if (by > maxHeight) maxHeight = by;

            count++;

            //fprintf(dp, "{ \"x\": %f, \"y\": %f, \"z\": %f, \"cid\": %d }%s \n", p.x - bounds_min[0], p.y - bounds_min[1], p.z - bounds_min[2], gid, i < num-1 ? "," : "");

            //pp.progress((double)i / num);

            //debugPrint("\n");
        }
    }

    {
        dtimer("kdtree - all");
        all.build();
    }
    {
        dtimer("kdtree - ground");
        ground.build();
    }

    num = count;

    unsigned int *cblocks = blocks.data();
    if (num > 0) {
        /*
        {
            //               //
            // Building fill //
            //               //
            dtimer("building fill");
            for (int i = 0; i < sxyz; i++) {
                unsigned int &cv = cblocks[i];
                int c = cv & 0xFF;
                switch (c) {
                case Classification::BUILDING:
                    int bx = i & msx;
                    int bz = (i >> psx) & msz;
                    int by = (i >> psx) >> psz;
                    //if (by > 130) c = Classification::WATER;
                    //*
                    int index = i - sxz;
                    int iy = by - 1;
                    while (iy > minHeight) {
                        assert(index > 0);
                        cblocks[index] = Classification::BUILDING;
                        index -= sxz;
                        iy--;
                    }

                    // Slanted roof search
                    pcln query_block_center[3] = {
                        bounds_min.x() + bz + 0.5,
                        bounds_min.y() + bx + 0.5,
                        bounds_min.z() + by + 0.5
                    };

                    const pcln radius = 2;
                    std::vector<std::pair<size_t, pcln> > indices;
                    RadiusResultSet<pcln, size_t> points(radius*radius, indices);
                    all.findRadius(query_block_center, points);

                    Eigen::Vector3d bcv;

                    bcv << query_block_center[0], query_block_center[1], query_block_center[2];

                    size_t buildingPoints = 0;
                    size_t slantedPoints = 0;
                    double avgAngle = 0;
                    double avgDot = 0;
                    for (size_t in = 0; in < points.size(); in++) {
                        std::pair<size_t, pcln> pair = points.m_indices_dists[in];
                        Point &p = all.getPoint(pair.first);
                        if (p.classification != Classification::BUILDING) continue;
                        Eigen::Vector3d pv(p.x, p.y, p.z);
                        Eigen::Vector3d rv = pv - bcv;
                        rv.normalize();
                        double dot = rv.dot(Eigen::Vector3d::UnitZ());
                        avgDot += abs(dot);
                        double angle = acos(dot) * 180 / M_PI;
                        avgAngle += abs(angle);
                        if (abs(angle - 45) < 30) slantedPoints++;
                        buildingPoints++;
                    }
                    avgAngle /= buildingPoints;
                    avgDot /= buildingPoints;
                    if (avgDot > 0.30) cv = (1 << 31) | 0x47d;

                    break;
                }
            }
        }
        */

        /*
        {
            //             //
            // Ground fill //
            //             //
            dtimer("ground fill");
            for (int iz = 0; iz < sz; iz++) {
                for (int ix = 0; ix < sx; ix++) {

                    int bx, by, bz;
                    bool found = false;

                    for (int iy = 0; iy < sy; iy++) {
                        if (cblocks[getBlockIndex(ix, iy, iz, sx, sxz)] == Classification::GROUND) {
                            bx = ix;
                            by = iy;
                            bz = iz;
                            found = true;
                            break;
                        }
                    }

                    if (!found) {
                        pcln query_block_center[3] = {
                            bounds_min.x() + iz + 0.5,
                            bounds_min.y() + ix + 0.5,
                            bounds_min.z() + 0 + 0.5
                        };

                        size_t ret_index;
                        pcln out_dist_sqr;
                        found = ground.findNearest(query_block_center, ret_index, out_dist_sqr);

                        if (found) {
                            Point &p = ground.getPoint(ret_index);
                            getBlockFromCoords(bounds_min.data(), p, bx, by, bz);
                            bx = ix;
                            bz = iz;
                        }
                    }

                    // Extend ground
                    if (found) {
                        for (int iy = 0; iy <= by; iy++) {
                            int index = getBlockIndex(bx, iy, bz, sx, sxz);
                            cblocks[index] = Classification::GROUND;
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
            for (int i = 0; i < sxyz; i++) {
                unsigned int &cv = cblocks[i];

                int bx = i & msx;
                int bz = (i >> psx) & msz;
                int by = (i >> psx) >> psz;

                if (cv) cv = classificationToBlock(cv);

                if (cv > 0) {
                    if (by > maxHeight) maxHeight = by;
                    if (by < minHeight) minHeight = by;
                    int colindex = bx + bz*sx;
                    int col = columns[colindex];
                    if (by > col) columns[colindex] = by;
                }

                //c |= (i%0xF) << 16;
            }
        }
    }

    //fprintf(dp, "] }");
    //fclose(dp);

    //("%d query took %ld ns\n", timer.tock().count());
    
    debugPrint("%d points found\n", num);
    debugPrint("%d points inside, %d points outside\n", count, num-count);

    maxHeight++;
    if (count == 0) maxHeight = -2;

    debug("%d points", num);
    debugPrint("%d points\n", count);

    {
        dtimer("serialization");

        amf::Serializer serializer;
        serializer << amf::AmfVector<unsigned int>(blocks);
        serializer << amf::AmfVector<unsigned int>(columns);

        amf::v8 arrays = serializer.data();

        if (!br.data) br.data = new amf::v8();
        amf::v8 *data = br.data;

        const int intSize = 4;
        size_t dataSize = 3 * intSize + arrays.size() + 1 * intSize;
        data->resize(dataSize);
        vassert(data->size() == dataSize, "%d %d", data->size(), dataSize);

        amf::u8 *p = data->data();
        writeInt(p, bx); p += intSize;
        writeInt(p, by); p += intSize;
        writeInt(p, bz); p += intSize;
        memcpy(p, arrays.data(), arrays.size()); p += arrays.size();
        writeInt(p, maxHeight); p += intSize;

        br.valid = true;

        debugPrint("%d bytes\n", data->size());

        debug("%d bytes", data->size());
    }
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


    debugPrint("%d max height\n", maxHeight);
    debugPrint("bx %d  bz %d  by %d\n", bx, bz, by);

    boxesCreated++;

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

    debugPrint("</pre></body></html>\n");

    //mapCloudHash.mutex.unlock();
    //br.mutex.unlock();
    //return status;
    //return "query";

    //printf("  GKOT %d, %d, %d   %d bytes\n", bx, by, bz, data.size());
}

void GKOTHandleDashboard(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
    mg_send_file(conn, "../www/GKOT/dashboard/dashboard.html");
}

void sendLiveImageHeader(struct mg_connection *conn, size_t size)
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

void GKOTHandleDashboardBoxes(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
    MapImage img;
    renderBoxesUsed(img);
    sendLiveImageHeader(conn, img.size);
    int status = mg_write(conn, img.data, img.size);
    if (status == -1) plog("Debug box write error");
}

void GKOTHandleDashboardMapClouds(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
    MapImage img;
    renderMapClouds(img);
    sendLiveImageHeader(conn, img.size);
    int status = mg_write(conn, img.data, img.size);
    if (status == -1) plog("Debug box write error");
}

void GKOTHandleDashboardStats(struct mg_connection *conn, void *cbdata, const mg_request_info *info)
{
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


int
GKOTHandler(struct mg_connection *conn, void *cbdata)
{
    //mapCloudHash.mutex.lock();

    const mg_request_info *info = mg_get_request_info(conn);

    //bool inside = lasreader->inside_rectangle(462000.00, 101000.00, 462000.00 + 250, 101000.00 + 600);

    //const char* status;

    if (strcmp(info->request_uri, "/GKOT/box") == 0) {
        GKOTHandleBox(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/GKOT/dashboard/") == 0) {
        GKOTHandleDashboard(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/GKOT/dashboard/boxes.png") == 0) {
        GKOTHandleDashboardBoxes(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/GKOT/dashboard/mapClouds.png") == 0) {
        GKOTHandleDashboardMapClouds(conn, cbdata, info);
    } else if (strcmp(info->request_uri, "/GKOT/dashboard/stats.json") == 0) {
        GKOTHandleDashboardStats(conn, cbdata, info);
    } else if (startsWith(info->request_uri, "/GKOT/dashboard/")) {
        if (strstr(info->request_uri, "..") != NULL) return 0;
        char path[1024];
        int stored = _snprintf(path, sizeof(path) - 1, "../www/%s", info->request_uri); path[stored] = 0;
        mg_send_file(conn, path);
    } else if (strcmp(info->request_uri, "/GKOT/debug") == 0) {
        GKOTHandleDebug(conn, cbdata, info);
    } else {
        return 0;
    }

    requestsServed++;

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



    /* Add handler EXAMPLE_URI, to explain the example */
    mg_set_request_handler(server, "/GKOT/", GKOTHandler, 0);

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
