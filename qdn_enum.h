#ifndef _QDN_ENUM_H_
#define _QDN_ENUM_H_

/// A class that creates a native enum object, but also assigns human readable strings to each
/// value, so that they can be printed out symbolically, and also the class can also parse strings
/// and convert to enums.
///
/// Caveat: some names are not allowed as enum values. For example, "Exception" is not allowed.
/// Example:
///
///  SMART_ENUM(Colors_e, Red, White, Blue, Yellow=7);
///
///  try {
///    Colors_e x = Colors_e::Red;
///    cout << x.toString() << endl;
///    x = Colors_e::fromString("White");
///  }
///  catch(Colors_e::Exception& ex) {
///    cout << ex.what() << endl;
///  }

#include <map>
#include <sstream>
#include <vector>
#include <algorithm>
#include <mutex>



#if __cpp_exceptions
#define MUTEX_T std::mutex
#define NOEXCEPT noexcept
#else
#define throw
#define NOEXCEPT
#define MUTEX_T int
#endif


// #define quoter(first,...) #first,quoter(__VA_ARGS__)

#define QDN_ENUM(EnumName, ...)                                   \
class EnumName                                                      \
{                                                                   \
public:                                                             \
    /*EnumName() NOEXCEPT : value(0) {}   */                            \
    EnumName() = default;                               \
    constexpr EnumName(int x) NOEXCEPT : value(x) {}                          \
    EnumName(const std::string&x) : value(fromString(x)) {}\
public:                                                             \
    enum { __VA_ARGS__ };                                           \
private:                                                            \
    static void initMap(std::map<int, std::string>& tmp)            \
    {                                                               \
        using namespace std;                                        \
        int val = 0;                                                \
        string buf_1, buf_2, str = #__VA_ARGS__;                    \
        replace(str.begin(), str.end(), '=', ' ');                  \
        stringstream stream(str);                                   \
        vector<string> strings;                                     \
        while (getline(stream, buf_1, ','))                         \
            strings.push_back(buf_1);                               \
        for (vector<string>::iterator it = strings.begin();         \
            it != strings.end();                                    \
            ++it)                                                   \
        {                                                           \
            buf_1.clear(); buf_2.clear();                           \
            stringstream localStream(*it);                          \
            localStream >> buf_1 >> buf_2;                          \
            if (buf_2.size() > 0)                                   \
                 val = atoi(buf_2.c_str());                         \
            tmp[val++] = buf_1;                                     \
        }                                                           \
    }                                                               \
    int value;                                                      \
public:                                                             \
    constexpr operator int() const { return value; }                          \
    std::string toString(void) const {                              \
      return toString(value);                                       \
    }                                                               \
    static std::string toString(int aInt)                           \
    {                                                               \
      return nameMap()[aInt];                                       \
    }                                                               \
    static EnumName fromString(const std::string& s)                \
    {                                                               \
        auto it = find_if(nameMap().begin(), nameMap().end(),       \
           [s](const std::pair<int, std::string>& p) {              \
            return p.second == s;                                   \
        });                                                         \
        if (it == nameMap().end()) {                                \
            /*value not found*/                                     \
            throw EnumName::Exception("'" + s +                     \
             "' not found in allowed values for enum " #EnumName ); \
            return 0;                                               \
        } else {                                                    \
            return EnumName(it->first);                             \
        }                                                           \
    }                                                               \
    static bool fromString(const std::string& s, EnumName& e)       \
    {                                                               \
        auto it = find_if(nameMap().begin(), nameMap().end(),       \
           [s](const std::pair<int, std::string>& p) {              \
            return p.second == s;                                   \
        });                                                         \
        if (it == nameMap().end()) {                                \
            return false;                                           \
        } else {                                                    \
            e = EnumName(it->first);                                \
            return true;                                            \
        }                                                           \
    }                                                               \
    class Exception : public std::runtime_error {                   \
    public: Exception(const std::string& msg)                       \
        : std::runtime_error(msg) {}                                \
    };                                                              \
    static std::map<int, std::string>& nameMap() {                  \
        static std::map<int, std::string> nameMap0;                 \
        /*static MUTEX_T mutex;                                    \
        std::lock_guard<MUTEX_T> g(mutex);   */                     \
        if (nameMap0.size() == 0) initMap(nameMap0);                \
        return nameMap0;                                            \
    }                                                               \
    static std::vector<EnumName> allValues() {                      \
        std::vector<EnumName> v;                                    \
        auto& m = nameMap();                                        \
        for (auto it = m.begin(); it != m.end(); ++it) {            \
            v.push_back(it->first);                                 \
        }                                                           \
        return v;                                                   \
    }                                                               \
    bool operator<(const EnumName a) const {                        \
        return (int)*this < (int)a;                                 \
    }                                                               \
};

//inline std::ostream& operator<<(std::ostream& s, const EnumName& e) { return s << e.toString(); }


#endif // _QDN_ENUM_H_
