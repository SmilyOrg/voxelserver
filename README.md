# voxelserver

# API

## `/gkot/origin.json`

Returns some information about the provided origin coordinates in JSON format. Note that the heights returned here are only ground heights and don't include any trees or buildings that might extend above ground.

### `tmx=[float]&tmy=[float]`

Origin coordinates in the D96/TM coordinate system.

### Example

The request for origin information around Castle Hill 

`/gkot/origin.json?tmx=462000&tmy=101000`

returns the following JSON info:

```
{
    "tmx" : 462000,
    "tmy" : 101000,
    "tmz" : 298.18,
    "zmax" : 368.03,
    "zmin" : 285.81,
    "zrange" : 82.21999999999997
}
```

* `tmx`, `tmy` are the same coordinates as in the parameters echoed back.

* `tmz` is the above sea level ground height retrieved from the DMR (Digital Model Relief) data or `-1` if height data was unavailable at the provided location.

* `zmin`, `zmax`, `zrange` are the minimum height, maximum height and the range between them found in the approximate 1km neighborhood around the provided point. You can use this to estimate the necessary chunk height in the next steps.


## `/gkot/box`

Generates a chunk of the world based on the provided parameters and returns it in binary format.

### `format=[amf|raw]` 

Output binary format, you should use `raw` as it is the most straightforward. Defaults to `amf` for compatibility reasons. See below for details.

### `tmx=[float]&tmy=[float]&tmz=[float]`

Origin coordinates in the [D96/TM](http://www.e-prostor.gov.si/si/zbirke_prostorskih_podatkov/drzavni_koordinatni_sistem/horizontalni_drzavni_koordinatni_sistem_d96tm/) real world coordinate system. The origin is where the block at `0, 0, 0` in world block coordinates is placed in the real world. `tmx` and `tmy` define a horizontal plane with `tmz` being the vertical offset. Note that this is in contrast with _block_ coordinates, where `x` and `z` define a horizontal plane with `y` being the vertical coordinate.

For example, setting `tmx` to `462000` and `tmy` to `101000` positions the block origin near the Ljubljana Castle. Using `290` for `tmz` the origin is placed underground a bit below the base of Castle Hill.

### `sx=[integer]&sy=[integer]&sz=[integer]`

Size of the chunk in multiples of one block. Each block is a cube with a side of 1m. `sx` and `sz` define the horizontal extent and `sy` defines the height of the chunk. A chunk usually forms a vertical cuboid or rectangular prism, where `sy` is a lot bigger than `sx` and `sz`.

It's best to use the same values for `sx` and `sz`. Due to implementation reasons, `sx` and `sz` **must** be a power of two, e.g. `16`, `32`, `64`, etc.

For example, providing `sx=16&sy=128&sz=16` will return a chunk of a square 16m×16m piece of land, extending upwards for 128 meters.

### `x=[integer]&y=[integer]&z=[integer]`

Location of the chunk in world block coordinates. You can use these parameters in a tile-based system by providing multiples of `sx`, `sy`, `sz`. Note that `sx` and `sz` define the horizontal plane and `sy` defines the vertical offset.

For example, providing `x=64&y=0&z=32` will return a chunk that has its bottom south-west block positioned at `64, 0, 32` in block coordinates. Querying with the same parameters except for location being `x=48&y=0&z=32`, `x=80&y=0&z=32`, `x=64&y=0&z=16` or `x=64&y=0&z=48` and with a horizontal chunk size of 16m×16m will return the neighboring chunks so that combined they form a system of tiles. This is useful for dynamic loading and unloading of parts of the world.

### `debug=[true|false]`

Defaults to `false`. If `true`, the server doesn't return the box in binary format, but rather outputs HTML information about the chunk suitable for viewing in a browser, including all the provided parameters and an exploded view of all the chunk layers.

### `transform=[true|false]`

Defaults to `false`. If `true`, the server transforms the point height right after loading it using the following integrated function.

```cpp
if (z <= threshold) {
    z *= scaleBelow;
} else {
    z = (z - threshold) * scaleAbove;
}
```

`z` is the height of the point above sea level, `threshold` is the inflection point of the height scaling and `scaleBelow` and `scaleAbove` are the scalars of the two different regions. These parameters are configurable via the `--transform-threshold`, `--transform-scale-below` and `--transform-scale-above` command line switches and are global across all requests that use `transform=true`. The default values scale all points in the interval `[0, 500]` to `[0, 200]` and `(500, 2864]` to `(200, 256]`.

### Example

`/gkot/box?format=raw&debug=true&tmx=462000&tmy=101000&tmz=290&x=64&y=0&z=32&sx=16&sy=128&sz=16`

Outputs the debug HTML view of a 128m high chunk of Castle Hill with a land footprint of 16m×16m near Ljubljana Castle. Omit `debug=true` to download the raw binary output.

# Output Formats

## `raw`

The binary format consists of series of 32-bit signed and unsigned integers in big endian byte order.

```
0 ................ 31 bits
 ___________________
[ chunk X           ] signed
[ chunk Y           ] signed
[ chunk Z           ] signed
[ max height        ] signed
[ blocks length     ] unsigned
[ blocks[0]         ] unsigned
[ blocks[1]         ] unsigned
[ blocks[2]         ] unsigned
     ...
[ blocks[length-1]  ] unsigned
[ columns length    ] unsigned
[ columns[0]        ] unsigned
[ columns[1]        ] unsigned
[ columns[2]        ] unsigned
     ...
[ columns[length-1] ] unsigned
 -------------------
```

* `chunk X`, `chunk Y`, `chunk Z` are the computed chunk coordinates of the provided parameters. For example, if you provide `x=64&y=0&z=32&sx=16&sy=128&sz=16` as the parameters, the returned coordinates are going to be `4, 0, 2`.

* `max height` is the maximum height that the non-air blocks reach in the chunk in meters. This will report the height of the top edge of the highest non-air block in the chunk.
    
    **Note:** A special value of `-2` signifies that this chunk is empty (i.e. full of air) and that you should not expect the `blocks` array to hold any blocks.

* `blocks length` is an unsigned integer defining how many blocks are to come after this field. This should equal either `sx*sy*sz` or zero.

* `blocks` is an array of `blocks length` unsigned integers representing all of the individual blocks in the chunk.

    Currently the byte format of the block is as follows:
    ```
    unsigned integer with 4 bytes
    
    [ 00 00 0D BB ]

    B - Minecraft Block ID [0, 255]
    D - Minecraft Block Data [0, F]
    0 - Always zero
    ```

    The index of a block is computed as follows: `bx + bz*sx + by*sx*sz`, where `bx`, `by` and `bz` are the coordinates of the block and `sx` and `sz` is the horizontal size of the chunk.

* `columns length` is an unsigned integer defining how many columns are to come after this field. This should equal `sx*sz`.

* `columns` is an array of `columns length` unsigned integers representing columns in the chunk. The number for each column defines the height or `by` value of the highest non-air block in the column.

    The index of a column is computed as follows: `bx + bz*sx`, where `bx` and `bz` are the horizontal coordinates and `sx` is the size of the chunk along the X axis.


## `amf`

The binary format is a mix of integers and AMF3 (ActionScript Message Format v3) encoded arrays. 

For this format you also need to provide a `worldHash` parameter with an integer ID that the server then echoes back in the contents.

Additionally, the contents of the chunk are vertically squeezed together into just the required height for this format, which is what the `required Y offset` and `required Y size` values report.

```
 ___________________
[ world hash        ] unsigned 32-bit int
[ required Y offset ] signed 32-bit int
[ required Y size   ] signed 32-bit int
[ chunk X           ] signed 32-bit int
[ chunk Y           ] signed 32-bit int
[ chunk Z           ] signed 32-bit int
[ blocks            ] AMF3 encoded Vector.<uint>
[ columns           ] AMF3 encoded Vector.<uint>
[ max height        ] signed 32-bit int
 -------------------
```

* `chunk X`, `chunk Y`, `chunk Z`, `max height` are the same as in the `raw` format above.

* `required Y offset` is the Y coordinate (height) from where the actual contents of the `blocks` Vector begin. Everything below that is assumed to be underground or underwater.

* `required Y size` is the height of the stored blocks, everything above the offset + size is assumed to be air and is not stored.

* `blocks` and `columns` are AMF3-encoded Vectors of unsigned integers. See the AMF3 specification for Vector storage or decode with an appropriate decoder.