#include "Gerenciador.h"

#include <algorithm>
#include <iostream>

void Gerenciador::comandos(Grafo* grafo) {
    std::cout << "Digite uma das opcoes abaixo e pressione enter:" << std::endl
              << std::endl;
    std::cout << "(a) Fecho transitivo direto de um no;" << std::endl;
    std::cout << "(b) Fecho transitivo indireto de um no;" << std::endl;
    std::cout << "(c) Caminho minimo (Djikstra);" << std::endl;
    std::cout << "(d) Caminho minimo (Floyd);" << std::endl;
    std::cout << "(e) Arvore Geradora Minima (Algoritmo de Prim);" << std::endl;
    std::cout << "(f) Arvore Geradora Minima (Algoritmo de Kruskal);" << std::endl;
    std::cout << "(g) Arvore de caminhamento em profundidade;" << std::endl;
    std::cout << "(h) Raio, diametro, centro e periferia do grafo;" << std::endl;
    std::cout << "(0) Sair;" << std::endl
              << std::endl;

    char resp;
    std::cin >> resp;
    switch (resp) {
        case 'a':
            {
                char id_no = getIdEntrada();
                std::vector<char> fecho_transitivo_direto = grafo->fechoTransitivoDireto(id_no);
                std::cout << "Metodo de impressao em tela nao implementado" << std::endl
                          << std::endl;

                if (perguntaImprimirArquivo("fecho_trans_dir.txt")) {
                    std::cout << "Metodo de impressao em arquivo nao implementado" << std::endl
                              << std::endl;
                }

                break;
            }
        case 'b':
            {
                char id_no = getIdEntrada();
                std::vector<char> fecho_transitivo_indireto = grafo->fechoTransitivoIndireto(id_no);
                std::cout << "Metodo de impressao em tela nao implementado" << std::endl
                          << std::endl;

                if (perguntaImprimirArquivo("fecho_trans_indir.txt")) {
                    std::cout << "Metodo de impressao em arquivo nao implementado" << std::endl;
                }

                ;
                break;
            }
        case 'c':
            {
                char id_no_1 = getIdEntrada();
                char id_no_2 = getIdEntrada();
                std::vector<char> caminho_minimo_dijkstra = grafo->caminhoMinimoDijkstra(id_no_1, id_no_2);
                std::cout << "Metodo de impressao em tela nao implementado" << std::endl
                          << std::endl;

                if (perguntaImprimirArquivo("caminho_minimo_dijkstra.txt")) {
                    std::cout << "Metodo de impressao em arquivo nao implementado" << std::endl;
                }

                break;
            }
        case 'd':
            {
                char id_no_1 = getIdEntrada();
                char id_no_2 = getIdEntrada();
                std::vector<char> caminho_minimo_floyd = grafo->caminhoMinimoFloyd(id_no_1, id_no_2);
                std::cout << "Metodo de impressao em tela nao implementado" << std::endl
                          << std::endl;

                if (perguntaImprimirArquivo("caminho_minimo_floyd.txt")) {
                    std::cout << "Metodo de impressao em arquivo nao implementado" << std::endl;
                }

                break;
            }
        case 'e':
            {
                int tam;
                std::cout << "Digite o tamanho do subconjunto: ";
                std::cin >> tam;

                if (tam > 0 && tam <= grafo->ordem) {
                    std::vector<char> ids = getConjuntoIds(grafo, tam);
                    Grafo* arvore_geradora_minima_prim = grafo->arvoreGeradoraMinimaPrim(ids);
                    std::cout << "Metodo de impressao em tela nao implementado" << std::endl
                              << std::endl;

                    if (perguntaImprimirArquivo("agm_prim.txt")) {
                        std::cout << "Metodo de impressao em arquivo nao implementado" << std::endl;
                    }

                    delete arvore_geradora_minima_prim;

                } else {
                    std::cout << "Valor invalido" << std::endl;
                }

                break;
            }
        case 'f':
            {
                int tam;
                std::cout << "Digite o tamanho do subconjunto: ";
                std::cin >> tam;

                if (tam > 0 && tam <= grafo->ordem) {
                    std::vector<char> ids = getConjuntoIds(grafo, tam);
                    Grafo* arvore_geradora_minima_kruskal = grafo->arvoreGeradoraMinimaKruskal(ids);
                    std::cout << "Metodo de impressao em tela nao implementado" << std::endl
                              << std::endl;

                    if (perguntaImprimirArquivo("agm_kruskal.txt")) {
                        std::cout << "Metodo de impressao em arquivo nao implementado" << std::endl;
                    }

                    delete arvore_geradora_minima_kruskal;

                } else {
                    std::cout << "Valor invalido" << std::endl;
                }

                break;
            }
        case 'g':
            {
                char id_no = getIdEntrada();
                Grafo* arvore_caminhamento_profundidade = grafo->arvoreCaminhamentoProfundidade(id_no);
                std::cout << "Metodo de impressao em tela nao implementado" << std::endl
                          << std::endl;

                if (perguntaImprimirArquivo("arvore_caminhamento_profundidade.txt")) {
                    std::cout << "Metodo de impressao em arquivo nao implementado" << std::endl;
                }

                delete arvore_caminhamento_profundidade;
                break;
            }
        case 'h':
            {
                int raio = grafo->raio();
                int diametro = grafo->diametro();
                std::vector<char> centro = grafo->centro();
                std::vector<char> periferia = grafo->periferia();

                std::cout << "Metodo de impressao em tela nao implementado" << std::endl
                          << std::endl;

                if (perguntaImprimirArquivo("raio_diametro_centro_periferia.txt")) {
                    std::cout << "Metodo de impressao em arquivo nao implementado" << std::endl;
                }

                break;
            }
        case '0':
            {
                exit(0);
            }
        default:
            {
                std::cout << "Opção inválida" << std::endl;
            }
    }

    comandos(grafo);
}

char Gerenciador::getIdEntrada() {
    std::cout << "Digite o id de um no: ";
    char id;
    std::cin >> id;
    std::cout << std::endl;
    return id;
}

std::vector<char> Gerenciador::getConjuntoIds(Grafo* grafo, int tam) {
    std::vector<char> ids = {};
    while ((int) ids.size() < tam) {
        char id_no = getIdEntrada();
        bool existe = false;
        for (No* no : grafo->lista_adj) {
            if (no->id == id_no) {
                existe = true;
                break;
            }
        }

        if (!existe) {
            std::cout << "Vertice nao existe" << std::endl;
        } else {
            bool repetido = find(ids.begin(), ids.end(), id_no) != ids.end();
            if (repetido) {
                std::cout << "Valor repetido" << std::endl;
            } else {
                ids.push_back(id_no);
            }
        }
    }

    return ids;
}

bool Gerenciador::perguntaImprimirArquivo(std::string nome_arquivo) {
    std::cout << "Imprimir em arquivo externo? (" << nome_arquivo << ")" << std::endl;
    std::cout << "(1) Sim;" << std::endl;
    std::cout << "(2) Nao." << std::endl;
    int resp;
    std::cin >> resp;
    std::cout << std::endl;

    switch (resp) {
        case 1:
            return true;
        case 2:
            return false;
        default:
            std::cout << "Resposta invalida" << std::endl;
            return perguntaImprimirArquivo(nome_arquivo);
    }
}
