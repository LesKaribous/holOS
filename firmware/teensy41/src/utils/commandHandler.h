#pragma once
#include "utils/geometry.h"
#include "commandHandler.h"

#include <map>
#include <vector>
#include <Arduino.h>

using arg_t = String;
using args_t = std::vector<String>;
using command_func_ptr = void (*)(const args_t& args);

class Command {
public:
    friend class CommandHandler;
    friend class Interpreter;
    friend class Program;
    Command(){};

    inline String& getSyntax(){return syntax;}
    inline String& getDesc(){return description;}

protected:
    Command(const String& synt, const String& desc, int argsCounts, command_func_ptr func);

    String syntax;
    String description;
    int numberOfArguments;

private:
    command_func_ptr callback = nullptr;
};

class CommandHandler {
protected :
   static void execute(const String& command, const args_t& args);

public:
    friend class Program;
    static void registerCommand(const String& syntax, const String& description, command_func_ptr func);
    static std::vector<String> extractArguments(const String& args);
    static bool hasCommand(const String& command);
    static std::map<String, Command>& getCommands();
    // Direct dispatch for trivial "name(args)" payloads that don't need
    // variable resolution or multi-statement scripting. Args are passed
    // through verbatim (the command callbacks themselves do .toFloat() /
    // .toInt() conversion on numeric args, so raw literals are fine).
    // Bypasses Interpreter + Program + Expression, saving ~1 KB stack
    // per nested call. Returns true if the command exists and ran.
    static bool dispatchDirect(const String& name, const args_t& args);
};

