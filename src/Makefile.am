AM_CPPFLAGS = \
	$(all_includes) \
	-I$(top_srcdir)/include \
	-I$(top_builddir) \
	$(NULL)

AM_CFLAGS = \
	-Wall \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(LIBOSMOVTY_CFLAGS) \
	$(COVERAGE_CFLAGS) \
	$(NULL)

AM_LDFLAGS = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(COVERAGE_LDFLAGS) \
	$(NULL)

# Libraries
SUBDIRS = \
	libcommon \
	libcommon-cs \
	libbsc \
	libfilter \
	$(NULL)

# Programs
SUBDIRS += \
	utils \
	ipaccess \
	osmo-bsc \
	$(NULL)
