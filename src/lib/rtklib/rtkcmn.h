#ifndef RTKCOMMON_H_
#define RTKCOMMON_H_

#include <stdint.h>
#include <math.h>

#ifndef PI
#define PI          3.141592653589793
#endif

#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */

uint32_t      rtk_crc24q(const uint8_t *buff, int len);
void           rtk_crc24q_init(uint32_t *crc);
void           rtk_crc24q_update(uint8_t c, uint32_t *crc);
unsigned int  getbitu(const unsigned char *buff, int pos, int len);
int           getbits(const unsigned char *buff, int pos, int len);
uint64_t      getbitlu(const unsigned char *buff, int pos, int len);
int64_t       getbitls(const unsigned char *buff, int pos, int len);

void          pos2ecef(const double *pos, double *r);
void          ecef2pos(const double *r, double *pos);
void          xyz2enu(const double *pos, double *E);
void          ecef2enu(const double *pos, const double *r, double *e);
void          enu2ecef(const double *pos, const double *e, double *r);
void          covenu(const double *pos, const double *P, double *Q);
void          covecef(const double *pos, const double *Q, double *P);
void          matmul(const char *tr, int n, int k, int m, double alpha,
                 const double *A, const double *B, double beta, double *C);


#endif
