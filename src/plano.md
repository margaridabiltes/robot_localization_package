## Filtro
3. Testar com várias formas de objetos ( circulo, triangulo, x)         -> **Miguel**
    * Círculo e dois círculos testado e funciona bem ✅                 -> **Miguel**
3. Adicionar incerteza da classificação na deteção do objeto            -> **Miguel**
    * Feito, mas não se nota diferença no output (menos com confidence a 0 em que o programa crasha, particulas ficam a zero)
    * Testar com confianças diferentes para objetos diferentes
3. Detetor consulta se o objeto é conhecido ou não                      -> **Miguel**
    * Feature_map do detetor é igual ao do filtro => isto n faz sentido
    * Tem de se criar um feature_map que só o detetor use, e nesse até se pode adicionar as confianças a cada objeto, para
    testar comportamentos diferentes
2. Fazer uma versão 3D                                                  -> **Biltes**
2. Ver o exemplo do héber pa também tirar dúvidas dos noises do motion

## Simulação
1. Importar modelo para o Webots
1. Ver do plug-in de RGB-D

## Deteção AI
1. Ler artigos 