#include "drake_helpers.hpp"

void get_system_graph(const drake::systems::Diagram<double> * diagram,const std::string & filePath){
    std::string GraphString = diagram -> GetGraphvizString(1);
    // std::string filePath = "graph.dot";

    // Create an output file stream
    std::ofstream outFile(filePath);

    // Check if the file stream is open
    if (outFile.is_open()) {
        // Write the string to the file
        outFile << GraphString;

        // Close the file stream
        outFile.close();

        std::cout << "String successfully written to file: " << filePath << std::endl;
        std::cout << "View using: `dot -Tpng " << filePath << " -o diagram.png`" << std::endl;
    } else {
        std::cerr << "Error opening the file: " << filePath << std::endl;
    }
}