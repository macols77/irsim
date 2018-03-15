AC_PREREQ(2.50)

AC_INIT(twodeepuck, 0.0.0)
AM_INIT_AUTOMAKE(twodeepuck, 0.0.0)
AM_CONFIG_HEADER(config.h)

dnl libreadline is not used AFAIK
dnl AC_CHECK_HEADERS(readline/readline.h, [], AC_MSG_ERROR([You need to install readline.h (probably in a readline-dev package)]))

dnl check for programs
AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_RANLIB

dnl c flags tuning 

CFLAGS="-O3"
CXXFLAGS="-O3"

AC_ARG_ENABLE( [amd64], 
	        AC_HELP_STRING([--enable-amd64], [enable amd64 optimizations]), 
		[CFLAGS=" -O3 -march=nocona -pipe -fomit-frame-pointer -fforce-addr -frerun-cse-after-loop -frerun-loop-opt -falign-functions=4 -maccumulate-outgoing-args -ffast-math -m32", CXXFLAGS=" -O3 -march=nocona -pipe -fomit-frame-pointer -fforce-addr -frerun-cse-after-loop -frerun-loop-opt -falign-functions=4 -maccumulate-outgoing-args -ffast-math -m32"])

AC_ARG_ENABLE( [i686], 
	        AC_HELP_STRING([--enable-i686], [enable i686 optimizations]), 
		[CFLAGS=" -O3 -march=i686 -pipe -fomit-frame-pointer -fforce-addr -frerun-cse-after-loop -frerun-loop-opt -falign-functions=4 -maccumulate-outgoing-args -ffast-math ", CXXFLAGS=" -O3 -march=i686 -pipe -fomit-frame-pointer -fforce-addr -frerun-cse-after-loop -frerun-loop-opt -falign-functions=4 -maccumulate-outgoing-args -ffast-math"])

AC_ARG_ENABLE( [athlonxp], 
	        AC_HELP_STRING([--enable-athlonxp], [enable Athlon XP optimizations]), 
		[CFLAGS=" -O3 -march=athlon-xp -mfpmath=sse -mmmx -m3dnow -msse -pipe -fomit-frame-pointer -fforce-addr -frerun-cse-after-loop -frerun-loop-opt -falign-functions=4 -maccumulate-outgoing-args -ffast-math ", CXXFLAGS=" -O3 -march=athlon-xp -mfpmath=sse -mmmx -m3dnow -msse -pipe -fomit-frame-pointer -fforce-addr -frerun-cse-after-loop -frerun-loop-opt -falign-functions=4 -maccumulate-outgoing-args -ffast-math"])

 AC_ARG_ENABLE( [debug], 
		AC_HELP_STRING([--enable-debug], [enable debugging]), 
		[CFLAGS=" -g", CXXFLAGS=" -g"])

 AC_ARG_ENABLE( [profile], 
		AC_HELP_STRING([--enable-profile], [enable profiling]), 
		[CFLAGS=" -g -pg", CXXFLAGS=" -g -pg"])

AC_ARG_ENABLE( [cluster], 
	        AC_HELP_STRING([--enable-cluster], [enable Opteron optimizations]), 
		[CFLAGS=" -O3 -march=opteron -pipe -fomit-frame-pointer -fforce-addr -frerun-cse-after-loop -frerun-loop-opt -falign-functions=4 -maccumulate-outgoing-args -ffast-math -fprefetch-loop-arrays", CXXFLAGS=" -O3 -march=opteron -pipe -fomit-frame-pointer -fforce-addr -frerun-cse-after-loop -frerun-loop-opt -falign-functions=4 -maccumulate-outgoing-args -ffast-math -fprefetch-loop-arrays"])


AC_CONFIG_FILES([ \
  Makefile        
  objects/Makefile
  sensors/Makefile 
  controllers/Makefile 
  collisions/Makefile
  actuators/Makefile 
  arenas/Makefile 
  renders/Makefile 
  renders/drawstuff/Makefile
  experiments/Makefile
	populations/Makefile
	fitnessfunctions/Makefile
])

AC_OUTPUT
