namespace std
{
    class ios_base /Abstract/
    {
%TypeHeaderCode
#include <iostream>
%End
    
    public:

        // Define openmode flags
        enum openmode
        {
            binary = 0x01,
            app    = 0x02,
            in     = 0x04,
            out    = 0x08,
            trunc  = 0x10,
            ate    = 0x20
        };
    };
};
