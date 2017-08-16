namespace std 
{
    class exception 
    {
%TypeHeaderCode
#include <exception>
%End
    public:
        exception() throw();

        virtual ~exception() throw();

        virtual const char* what() const throw();
    };
};