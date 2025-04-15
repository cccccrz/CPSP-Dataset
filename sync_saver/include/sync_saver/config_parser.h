#pragma once
#include <map>
#include <string>
#include <fstream>

class ConfigParser {
public:
    explicit ConfigParser(const std::string& filepath) {
        parse(filepath);
    }

    std::string get(const std::string& key, const std::string& default_val = "") const;

    double get_double(const std::string& key, double default_val = 0.0) const;

private:
    void parse(const std::string& filepath);

    static std::string trim(const std::string& s);

    std::map<std::string, std::string> data_;
};