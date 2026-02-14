#ifndef LAZBridge_h
#define LAZBridge_h

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PPLOpaqueLAZWriter PPLOpaqueLAZWriter;

PPLOpaqueLAZWriter *pp_laz_writer_create(
    const char *path_utf8,
    double scale_x,
    double scale_y,
    double scale_z,
    double offset_x,
    double offset_y,
    double offset_z,
    uint32_t chunk_size,
    const char *wkt_utf8
);

bool pp_laz_writer_write_point(
    PPLOpaqueLAZWriter *writer,
    double x,
    double y,
    double z,
    uint16_t red,
    uint16_t green,
    uint16_t blue,
    uint16_t intensity,
    uint8_t classification
);

bool pp_laz_writer_close(PPLOpaqueLAZWriter *writer);
const char *pp_laz_writer_last_error(PPLOpaqueLAZWriter *writer);
void pp_laz_writer_destroy(PPLOpaqueLAZWriter *writer);

#ifdef __cplusplus
}
#endif

#endif /* LAZBridge_h */
