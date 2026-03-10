LIBIEC_HOME=../..

PROJECT_BINARY_NAME = tc_tvtr
PROJECT_SOURCES += sv_publisher_example.c

INCLUDES += -I.

include $(LIBIEC_HOME)/make/target_system.mk
include $(LIBIEC_HOME)/make/stack_includes.mk

all:	$(PROJECT_BINARY_NAME)

include $(LIBIEC_HOME)/make/common_targets.mk

$(PROJECT_BINARY_NAME):	$(PROJECT_SOURCES) $(LIB_NAME)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(PROJECT_BINARY_NAME) $(PROJECT_SOURCES) $(INCLUDES) $(LIB_NAME) $(LDLIBS) -lrt -lpaho-mqtt3c

clean:
	rm -f $(PROJECT_BINARY_NAME)


