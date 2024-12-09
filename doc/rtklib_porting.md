## 1. RTKLIB  files

`v2.4.2 p13`

```bash
./src/convkml.c     (gnss数据->google earth kml) ------------------------------->remove
	extern int convkml(...)
./src/convrnx.c     (gnss数据->RINEX格式接口) ------------------------------->remove
	extern int convrnx(...)
./src/datum.c       (日本高程系统转换接口)  -------------------------------->remove
	extern int jgd2tokyo(...)
	extern int loaddatump(...)
	extern int tokyo2jgd(...)
./src/download.c    (平台文件IO实现,存储数据) -------------------------------->remove
	extern int execcmd_to(...)
	extern int dl_readurls(...)
	extern int dl_readstas(...)
	extern int dl_exec(...)
	extern void dl_test(...)
./src/ephemeris.c   (星历数据操作接口)
	extern void alm2pos(...)
	extern double eph2clk(...)
	extern void eph2pos(...)
	extern double geph2clk(...)
	extern void geph2pos(...)
	extern double seph2clk(...)
	extern void seph2pos(...)
	extern int satpos(...)
	extern int satposs(...)
./src/geoid.c       (大地水准面模型,from file or embedded)
	extern int opengeoid(...)   [from file]
	extern void closegeoid(...) [from file]
	extern double geoidh(...)
./src/ionex.c
	extern void readtec(...)    [from file]
	extern int iontec(...)      
./src/lambda.c      (最小二乘估计)
	extern int lambda(...)
./src/options.c     (文件IO或其他) --------------------------------------->remove
	extern opt_t *searchopt(...)
	extern int str2opt(...)
	extern int opt2str(...)
	extern int opt2buf(...)
	extern int loadopts(...)
	extern int saveopts(...)
	extern void resetsysopts(...)
	extern void getsysopts(...)
	extern void setsysopts(...)
./src/pntpos.c      (定位及校正接口)
	extern int ionocorr(...)
	extern int tropcorr(...)
	extern int pntpos(...)
./src/postpos.c     (数据后处理接口) ------------------------------------->remove
	extern int postpos(...)     [from file] 
./src/ppp_ar.c      (整周模糊度算法)
	extern int pppamb(...)
./src/ppp.c         (PPP单点定位接口)
	extern int dehanttideinel_(...)
	extern void pppoutsolstat(...)   [into file] 
	extern void tidedisp(...)
	extern int pppnx(...)
	extern void pppos(...)
./src/preceph.c     (相位偏移)
	extern void readsp3(...)    [from file] 
	extern int readsap(...)     [from file] 
	extern int readdcb(...)     [from file] 
	extern void satantoff(...)
	extern int peph2pos(...)
./src/qzslex.c      [parical remove]
	extern int lexupdatecorr(...)
	extern int lexreadmsg(...)    [from file] 
	extern void lexoutmsg(...)    [into file] 
	extern int lexconvbin(...)    [file trans] 
	extern int lexeph2pos(...)
	extern int lexioncorr(...)
./src/rcvraw.c
	extern int decode_gal_inav(...)
	extern int decode_bds_d1(...)
	extern int decode_bds_d2(...)
	extern int test_glostr(...)
	extern int decode_glostr(...)
	extern int decode_frame(...)
	extern int init_raw(...)
	extern void free_raw(...)
	extern int input_raw(...)
	extern int input_rawf(...)   [from file]
./src/rinex.c       (RINEX格式文件操作接口) [parical remove]
./src/rtcm.c
./src/rtcm2.c
./src/rtcm3.c
./src/rtcm3e.c
./src/rtkcmn.c
./src/rtklib.h
./src/rtkpos.c      (计算定位解)
./src/rtksvr.c      (RTK基站接口实现)
./src/sbas.c        (SBAS星基增强相关操作接口)
./src/solution.c    (定位解相关解析接口(nmea))
./src/stream.c      (平台通信IO实现,从usart/net口获取数据) ------------>remove
./src/streamsvr.c   (平台通信IO实现,基站发送数据接口)  ---------------->remove
./src/tle.c         (TLE两行轨道数据格式操作接口)  -------------------->remove
```

```c
2. rtkcmn.c and rtkcmn.h commented:
		extern int 			execcmd(const char *cmd)
		extern int 			expath(const char *path, char *paths[], int nmax)
		extern void 		createdir(const char *path)
3. 
            
		extern gtime_t 		timeget(void)
		extern void 		timeset(gtime_t t)
		extern unsigned int tickget(void)


```

