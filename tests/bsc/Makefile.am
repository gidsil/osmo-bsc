AM_CPPFLAGS = \
	$(all_includes) \
	-I$(top_srcdir)/include \
	$(NULL)

AM_CFLAGS = \
	-Wall \
	-ggdb3 \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(LIBOSMOABIS_CFLAGS) \
	$(LIBOSMOLEGACYMGCP_CFLAGS) \
	$(COVERAGE_CFLAGS) \
	$(NULL)

AM_LDFLAGS = \
	$(COVERAGE_LDFLAGS) \
	$(NULL)

EXTRA_DIST = \
	bsc_test.ok \
	$(NULL)

noinst_PROGRAMS = \
	bsc_test \
	$(NULL)

bsc_test_SOURCES = \
	bsc_test.c \
	$(top_srcdir)/src/osmo-bsc/osmo_bsc_filter.c \
	$(NULL)

bsc_test_LDADD = \
	$(top_builddir)/src/libbsc/libbsc.a \
	$(top_builddir)/src/libcommon-cs/libcommon-cs.a \
	$(top_builddir)/src/libcommon/libcommon.a \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(LIBOSMOVTY_LIBS) \
	$(LIBOSMOABIS_LIBS) \
	$(LIBOSMOLEGACYMGCP_LIBS) \
	$(LIBRARY_GSM) \
	-lrt \
	$(NULL)
