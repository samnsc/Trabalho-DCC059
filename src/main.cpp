#include "Gerenciador.h"

int main(int argc, char *argv[]) {
    Grafo *grafo = new Grafo();

    Gerenciador::comandos(grafo);

    return 0;
}
