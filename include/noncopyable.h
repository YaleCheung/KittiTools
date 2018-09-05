#ifndef NONCOPYABLE_HHH
#define NONCOPYABLE_HHH

class NonCopyable {
protected:
    NonCopyable() {};
    ~NonCopyable() {};
public:
    NonCopyable(const NonCopyable&) = delete;
    const NonCopyable& operator=(const NonCopyable&) = delete;
};
#endif
