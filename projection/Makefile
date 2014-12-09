TARGET := project

CXXFLAGS := -std=c++11 -O3 -DNDEBUG
LDFLAGS := 

PACKAGES := eigen3
LDLIBS += $(shell pkg-config --libs $(PACKAGES))
CXXFLAGS += $(shell pkg-config --cflags $(PACKAGES))


all : $(TARGET)
	

$(TARGET) : main.cc project.h project.cc
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ main.cc project.cc $(LDLIBS)

clean :
	$(RM) $(TARGET)
