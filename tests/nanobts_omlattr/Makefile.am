AM_CPPFLAGS = \
	$(all_includes) \
	-I$(top_srcdir)/include \
	$(NULL)

AM_CFLAGS = \
	-Wall \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(LIBOSMOABIS_CFLAGS) \
	$(NULL)

noinst_PROGRAMS = \
	nanobts_omlattr_test \
	$(NULL)

EXTRA_DIST = \
	nanobts_omlattr_test.ok \
	$(NULL)

nanobts_omlattr_test_SOURCES = \
	nanobts_omlattr_test.c \
	$(NULL)

nanobts_omlattr_test_LDADD = \
	$(top_builddir)/src/libbsc/libbsc.a \
	$(top_builddir)/src/libcommon/libcommon.a \
	$(top_builddir)/src/libcommon-cs/libcommon-cs.a \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(LIBOSMOABIS_LIBS) \
	$(NULL)
