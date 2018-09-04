#ifndef DATASETITERATOR_HHH
#define DATASETITERATOR_HHH

template<typename T>
class Iterator {
public:
    T& virtual begin() = 0;
    T& virtual next() = 0;
};

#endif // DATASETITERATOR_HHH
