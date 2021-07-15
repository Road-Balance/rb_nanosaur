#ifndef TEST_HPP_
#define TEST_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

class MyInfo
{
private:
    std::string m_name;
    int m_age;
public:
    MyInfo(std::string name_in="Swimming", int age_in = 20): m_name(name_in), m_age(age_in){}

    ~MyInfo(){}

    void get_info(){
        std::cout << "Ma Name is " <<  m_name << " and Age is " << m_age << std::endl;
    }
};


#endif