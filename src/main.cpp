#include "Svar.h"

auto highstitch=svar.import("svar_highstitch");

std::string getFolderPath(const std::string& path) {
    auto idx = std::string::npos;
    if ((idx = path.find_last_of('/')) == std::string::npos)
        idx = path.find_last_of('\\');
    if (idx != std::string::npos)
        return path.substr(0, idx);
    else
        return "";
}

int main(int argc,char** argv){
    svar.ParseMain(argc,argv);

    std::string lic         = svar.arg<std::string>("license","","If you wanna activate, set this");
    std::string task_file   = svar.arg<std::string>("task","","The task wanna process");
    std::string result      = svar.arg<std::string>("out","dom.jpg","The result file to save");

    if(svar.get("help",false))
        return svar.help();

    if(lic.size()){
        return highstitch["activate"](lic).as<bool>();
    }

    if(task_file.empty()){
        std::cerr<<"Please set task :\n  highstitch -task <your_task_file>\n";
        return 0;
    }

    auto task = sv::Svar::loadFile(task_file);

    task["parameters"]["topdir"] = getFolderPath(task_file);

    bool success = highstitch["stitch_task"](task,result).as<bool>();

    return success?0:-1;
}
