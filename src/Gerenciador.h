#ifndef __GERENCIADOR_H__
#define __GERENCIADOR_H__

#include <string>
#include <vector>

#include "Grafo.h"

namespace Gerenciador {
    void comandos(Grafo* grafo);
    char getIdEntrada();
    std::vector<char> getConjuntoIds(Grafo* grafo, int tam);
    bool perguntaImprimirArquivo(std::string nome_arquivo);
};  // namespace Gerenciador

#endif
