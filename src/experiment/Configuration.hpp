#ifndef METRONOME_CONFIGURATION_HPP
#define METRONOME_CONFIGURATION_HPP

namespace metronome {
class Configuration {
public:
    Configuration() : document{} {};

    Configuration(rapidjson::Document document) : document{std::move(document)} {
    }

    bool hasMember(const std::string& key) const {
        return document.HasMember(key.c_str());
    }

    std::string getString(const std::string& key) const {
        return std::string{document[key.c_str()].GetString()};
    }

    long long int getLong(const std::string& key) const {
        return document[key.c_str()].GetInt64();
    }

private:
    rapidjson::Document document;
};
}

#endif // METRONOME_CONFIGURATION_HPP
