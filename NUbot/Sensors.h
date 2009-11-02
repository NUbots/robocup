/*

*/

#include <vector>
#include <pthread.h>

using namespace std;

struct sensor_type 
{
    vector<float> data;
    vector<float> sd;
    bool isValid;
    bool isCalculated;
    long double timestamp;
    pthread_mutex_t mutex;
};

class Sensors
{
public:
    Sensors();
    ~Sensors();
private:
    
};


