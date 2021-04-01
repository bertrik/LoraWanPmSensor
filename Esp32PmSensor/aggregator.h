typedef struct {
    double sum;
    int count;
} sum_t;

class Aggregator {

private:
    sum_t *_items;
    int _size;

public:
    Aggregator(int size);
    ~Aggregator();

    void reset();
    void add(int index, double value);
    bool get(int index, double &value);
};

