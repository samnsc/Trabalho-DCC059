#ifndef __GERENCIADOR_H__
#define __GERENCIADOR_H__

#include <memory>
#include <string>
#include <vector>

#include "Grafo.h"

namespace Gerenciador {
    void comandos(std::unique_ptr<Grafo> grafo);
    char getIdEntrada();
    std::vector<char> getConjuntoIds(const Grafo& grafo, int tam);
    bool perguntaImprimirArquivo(std::string nome_arquivo);

    std::unique_ptr<Grafo> lerArquivo(const std::string& nome_arquivo);
};  // namespace Gerenciador

#endif
