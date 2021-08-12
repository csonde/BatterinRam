#include <cstdio>
#include <opencv2/highgui.hpp>
#include "Map.h"


int main(int argc, char** argv)
{
    if (argc != 2)
    {
        printf(" Usage: %s MapFileToParse\n", argv[0]);
        return -1;
    }
    namedWindow("BatteringRam", WINDOW_AUTOSIZE);
    try
    {
        Map map(argv[1], "BatteringRam", 640, 640);
        int frameCounter = 0;
        int stepFreq = 12;
        while (true)
        {
            frameCounter++;
            int key = waitKey(16);

            switch (key)
            {
                case '[':
                    map.activateNextSpot();
                    break;
                case ']':
                    map.activateNextSpot(false);
                    break;
                case 'p':
                    while (!map.planTrajectory(6000)) {};
                    break;
                case 's':
                    map.startStop();
                    break; 
                case 'r':
                    map.reset();
                    break;
                default:
                    break;
            }
            if (key == 27)
                break;

            if (frameCounter == stepFreq)
            {
                map.simulateStep();
                frameCounter = 0;
            }
            map.draw();
        }
    }
    catch (runtime_error& e)
    {
        printf("%s\n", e.what());
        return -1;
    }
    return 0;
}