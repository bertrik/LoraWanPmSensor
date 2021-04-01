#include <stdbool.h>
#include "aggregator.h"

Aggregator::Aggregator(int size)
{
    _size = size;
    _items = new sum_t[size];
}

Aggregator::~Aggregator()
{
    delete[] _items;
}

void Aggregator::add(int index, double value)
{
    if (index < _size) {
        _items[index].sum += value;
        _items[index].count++;
    } 
}

void Aggregator::reset(void)
{
    for (int i = 0; i < _size; i++) {
        _items[i].sum = 0.0;
        _items[i].count = 0;
    }
}

bool Aggregator::get(int index, double &value)
{
    if (index < _size) {
        if (_items[index].count > 0) {
            value = _items[index].sum / _items[index].count;
            return true;
        }
    }
    return false;
}


