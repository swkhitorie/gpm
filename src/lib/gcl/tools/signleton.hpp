
#ifndef __SIGNLETON_H_
#define __SIGNLETON_H_

namespace ESAF
{

template <class T>
class Singleton
{
  public:
    typedef T type;

  protected:
    Singleton();

  public:
    virtual ~Singleton() {}

  public:
    static T &instance();
    static T *instancePTR();

  protected:
    static T *singleInstance;
};  // class Singleton<T>

template <class T>
Singleton<T>::Singleton()
{}

template <class T>
T &Singleton<T>::instance()
{
    return *Singleton<T>::singleInstance;
}

template <class T>
T *Singleton<T>::instancePTR()
{
    return Singleton<T>::singleInstance;
}

template <class T>
T *Singleton<T>::singleInstance = new T();

}  // namespace ESAF

#endif  // signleton_H
