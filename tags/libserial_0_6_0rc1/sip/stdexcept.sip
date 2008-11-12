namespace std
{
  class logic_error :  std::exception 
  {
%TypeHeaderCode
#include <stdexcept>
%End
  public:
     explicit 
    logic_error(const std::string&  __arg);

    virtual 
    ~logic_error() throw();

     virtual const char* 
    what() const throw();
  };

  class runtime_error :  std::exception 
  {
  public:
%TypeHeaderCode
#include <stdexcept>
%End
    explicit 
    runtime_error(const std::string&  __arg);

    virtual 
    ~runtime_error() throw();

    virtual const char* 
    what() const throw();
  };

  class invalid_argument : std::logic_error 
  {
%TypeHeaderCode
#include <stdexcept>
%End
  public:
    explicit invalid_argument(const std::string&  __arg);
    virtual ~invalid_argument() throw() ;
%MethodCode
	        return ;
%End
  };
};