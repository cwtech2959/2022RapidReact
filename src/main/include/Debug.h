#pragma once

#include <variant>
#include <string>
#include <list>

namespace cwtech 
{

using DebugType = std::variant<double, std::string, bool>;

class DebugVariable
{
public:
    DebugVariable(std::string key, DebugType defaultValue);
    double GetNumber(DebugType defaultValue);
    std::string GetString(DebugType defaultValue);
    bool GetBoolean(DebugType defaultValue);
    //std::list<double> GetNumberList();
    //std::list<std::string> GetStringList();
    //std::list<bool> GetBooleanList();
    void PutNumber(double val);
    void PutString(std::string val);
    void PutBoolean(bool val);
    //void PutNumberList(std::list<double> val);
    //void PutStringList(std::list<std::string> val);
    //void PutBooleanList(std::list<bool> val);
private:
    std::string m_key;
    DebugType m_defaultValue;    
};

class Debug
{
public:
    Debug(std::string name, Debug* parent = nullptr);
    DebugVariable Variable(std::string name, DebugType defaultValue);
private:
    std::string m_name;
    std::string m_key;
};


}

