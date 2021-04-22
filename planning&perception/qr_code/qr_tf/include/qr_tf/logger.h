#pragma once

#include <vector>
#include <string>
#include <memory>
#include <sstream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <ctime>
#include <type_traits>

namespace lynx {

enum class Level: char {
    TRACE = 0,
    DEBUG,
    INFO,
    WARN,
    ERROR,
};

inline std::string level_names(Level l)
{
    static const std::string kLevelNames[] = {
        "T", "D", "I", "W", "E"
    };
    return kLevelNames[(int)l];
}

inline std::string level_color(Level l)
{
#ifdef __WIN32__
    static const std::string kLevelColor[] = {
        "", "", "", "", ""
    };
#else
    static const std::string kLevelColor[] = {
        " \33[34m", " \33[36m", " \33[32m", " \33[33m", " \33[31m",
    };
#endif
    return kLevelColor[(int)l];
}

inline std::string reset_color()
{
#ifdef __WIN32__
    return "";
#else
    return "\33[0m";
#endif
}

struct Logger {
    Logger(bool uc = false): use_color(uc) {}
    virtual ~Logger() {}
    virtual void write(const std::string&) = 0;
    
    bool used_color() const {return use_color;}
private:
    bool use_color {false};
};

class ConsoleLogger: public Logger
{
public:
    ConsoleLogger(): Logger(true) {}
    ~ConsoleLogger() {}
    void write(const std::string& content) override {
        std::cout << content;
    }
};

class FileLogger: public Logger
{
public:
    FileLogger(const std::string& file)
        :Logger(false), fout_(file, std::ios::out | std::ios::app)
    {}
    ~FileLogger() {}

    void write(const std::string& content) override {
        if (fout_.is_open()) {
            fout_.write(content.data(), content.size());
            fout_.flush();
        }
    }
private:
    std::ofstream     fout_;
};

class Log final
{
public:
    using logger_t = std::shared_ptr<Logger>;

    static Log *Instance() {
        static Log log;
        return &log;
    }

    void add_handle(logger_t logger) {handles_.emplace_back(logger);}
    void set_level(Level l) { level_ = l; }
    
    template<Level l, class... Args>
    void print(Args&&... to_print) {
        if (l < level_ || handles_.size() <= 0) return;
        char buf[max_buf_size];
        std::fill(buf, buf+max_buf_size, 0);
        auto now = std::time(nullptr);
        auto r = std::strftime(buf, max_buf_size, "<%H:%M:%S>", std::localtime(&now));
        auto endl = std::cout.widen('\n');
        
        std::string content;
        if (r)
            content += print(buf, std::forward<Args>(to_print)..., endl);
        else
            content += print(std::forward<Args>(to_print)..., endl);
        for (auto log: handles_) {
            log->write(level_string(l, log->used_color()) + content);
        }
    }

private:
    Log() {}
    Log(Level l): level_(l) {}
    ~Log() {}
    
    std::string level_string(Level l, bool color)
    {
        std::string level = " ";
        if (color) level += level_color(l);
        level += level_names(l);
        if (color) level += reset_color();
        return level + " ";
    }
    
    template<class T>
    std::string print(T&& only) {
        std::ostringstream oss;
        oss << only;
        return oss.str();
    }
    
    template<class Head, class... Tails>
    std::string print(Head&& head, Tails&&... tails) {
        return print(head) + " " +
                print(std::forward<Tails>(tails)...);
    }

private:
    enum {max_buf_size = 32};
    Level level_ {Level::DEBUG};
    std::vector<logger_t> handles_;
};

} // lynx

#define PRINT(l, ...)   \
    lynx::Log::Instance()->print<l>(__VA_ARGS__)

#define LTRACE(...)         PRINT(lynx::Level::TRACE, __VA_ARGS__)
#define LDEBUG(...)         PRINT(lynx::Level::DEBUG, __VA_ARGS__)
#define LINFO(...)          PRINT(lynx::Level::INFO,  __VA_ARGS__)
#define LWARN(...)          PRINT(lynx::Level::WARN,  __VA_ARGS__)
#define LERROR(...)         PRINT(lynx::Level::ERROR, __VA_ARGS__)

