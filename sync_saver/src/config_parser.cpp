#include "config_parser.h"

std::string ConfigParser::get(const std::string& key, const std::string& default_val = "") const {
    auto it = data_.find(key);
    return (it != data_.end()) ? it->second : default_val;
}

double ConfigParser::get_double(const std::string& key, double default_val = 0.0) const {
    return std::stod(get(key, std::to_string(default_val)));
}

void ConfigParser::parse(const std::string& filepath) {
    std::ifstream file(filepath);
    std::string line;
    std::string current_section;
    
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;

        if (line[0] == '[' && line.back() == ']') {
            current_section = line.substr(1, line.size()-2);
        } else {
            size_t eq_pos = line.find('=');
            if (eq_pos != std::string::npos) {
                std::string key = trim(line.substr(0, eq_pos));
                std::string value = trim(line.substr(eq_pos+1));
                data_[current_section + "." + key] = value;
            }
        }
    }
}

static std::string ConfigParser::trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t");
    size_t end = s.find_last_not_of(" \t");
    return (start == std::string::npos) ? "" : s.substr(start, end-start+1);
}