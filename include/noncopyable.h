#ifndef NONCOPYABLE_HHH
#define NONCOPYABLE_HHH

class NonCopyable {
public:
    NonCopyable(const NonCopyable&) = delete;
    NonCopyable& operator=(const NonCopyable&) = delete;
};
#endif
