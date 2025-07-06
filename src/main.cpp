#include <iostream>

#include "Gerenciador.h"

int main(int argc, char *argv[]) {
    // if argc is different from 2 it means either no parameter was given or multiple parameters were
    if (argc != 2) {
        std::cout << "Quantidade incorreta de parâmetros disponibilizados\n";
        return 1;
    }

    std::string filename = std::string(argv[1]);  // argv[0] is the program name, argv[1] is the first argument
    auto graph = Gerenciador::lerArquivo(filename);

    if (graph) {
        Gerenciador::comandos(std::move(graph));
    } else {
        std::cout << "O arquivo selecionado não foi encontrado ou é inválido.\n";
    }

    return 0;
}
