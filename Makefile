EXEC := calibration_server

DIRSRC := src/
DIROBJ := obj/
DIRHEA := include/

CXX := g++

CXXFLAGS := -Wall -O3 -std=c++0x
CXXFLAGS += `pkg-config --cflags opencv`

INCLUDES := -I$(DIRHEA)

LDLIBS := -lboost_system -lboost_thread -lpthread
LDLIBS += `pkg-config --libs opencv`

OBJS := $(subst $(DIRSRC), $(DIROBJ), $(patsubst %.cpp, %.o, $(wildcard $(DIRSRC)*.cpp)))

COLOR_FIN := \033[00m
COLOR_OK := \033[01;32m
COLOR_ERROR := \033[01;31m
COLOR_AVISO := \033[01;33m
COLOR_COMP := \033[01;34m
COLOR_ENL := \033[01;35m

.PHONY: all clean

all: info $(EXEC)

info:
	@echo -e '$(COLOR_AVISO)-------------------------$(COLOR_FIN)'
	@echo -e '$(COLOR_AVISO)Construyendo $(COLOR_COMP)$(EXEC)$(COLOR_FIN)'
	@echo -e '$(COLOR_AVISO)-------------------------$(COLOR_FIN)'
	@echo ''

$(EXEC): $(OBJS)
	@echo -e '$(COLOR_ENL)Enlazando$(COLOR_FIN): $(notdir $<)'
	@$(CXX) -o $@ $^ $(LDLIBS)
	@echo -e '$(COLOR_OK)Terminado.$(COLOR_FIN)'

$(DIROBJ)%.o: $(DIRSRC)%.cpp
	@echo -e '$(COLOR_COMP)Compilando$(COLOR_FIN): $(notdir $<)'
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	find . \( -name '*.log' -or -name '*~' \) -delete
	rm -f $(EXEC) $(DIROBJ)*
