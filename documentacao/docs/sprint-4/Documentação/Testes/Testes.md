---
sidebar_position: 1
custom_edit_url: null
title: "Testes referentes aos RNFs"
---

# Projeto de Detecção de Sujeira em Tubos - Testes de Funcionalidade e Validação de RNFs

## 1. Teste de Funcionalidade (Requisito Funcional)

### Teste: Operação do Robô (RF-1)

**Objetivo:** Verificar a funcionalidade do robô em identificar tubos que estão parcialmente ou totalmente obstruídos através de sua câmera.

**Procedimento:**
1. Configurar o robô com a câmera e o modelo YoloV8.
2. Iniciar o ciclo de verificação em um ambiente de teste com tubos obstruídos e limpos.
3. O robô deve capturar imagens, processá-las com o modelo YoloV8 e armazenar os resultados no backend.
4. Os usuários devem interagir com o sistema para visualizar as imagens e os resultados da detecção.

:::info
**Usuários Testados:** 4 usuários diferentes foram envolvidos no teste para garantir a consistência e a confiabilidade dos resultados.
:::

**Relatório:**
- **Interação 1:** O robô identificou corretamente os tubos entupidos. O usuário confirmou a precisão das imagens e dos resultados.
- **Interação 2:** Pequenas falhas na captura de algumas imagens, corrigidas na segunda tentativa. O relatório sugeriu melhorias na I.A.
- **Interação 3:** O usuário não teve dificuldades.
- **Interação 4:** Sem problemas significativos. Usuário sugeriu melhorias na interface gráfica para uma visualização mais intuitiva.

:::info
**Conclusões:**
- O robô funciona conforme esperado, identificando tubos entupidos com alta precisão.
:::

## 2. Testes de Validação de Requisitos Não Funcionais (RNFs)

### 2.1 RNF-1: Duração da Bateria

**Objetivo:** Verificar se a bateria do robô dura pelo menos um ciclo de verificação do reboiler.

**Procedimento:**
1. Carregar a bateria do robô completamente.
2. Iniciar um ciclo completo de verificação em um ambiente de teste.
3. Monitorar a duração da bateria durante todo o ciclo.

:::info
**Resultados:**
- A bateria durou entorno de 2 horas com o robô funcionando, superando o requisito.

**Conclusão:**
- A bateria do robô atende ao requisito de duração, garantindo operação contínua durante um ciclo de verificação.
:::

### 2.2 RNF-2: Latência do Controle

**Objetivo:** Verificar se a latência do controle do robô é menor que 200ms.

**Procedimento:**
1. Configurar o sistema de controle do robô.
2. Medir a latência de controle utilizando ferramentas de monitoramento de rede.
3. Repetir o teste em diferentes condições operacionais.

:::info
**Resultados:**
- A latência média foi de 150ms, com um máximo de 180ms.

**Conclusão:**
- A latência do controle está dentro do limite aceitável, atendendo ao requisito.
:::

### 2.3 RNF-4: Tempo de Resposta do Servidor

**Objetivo:** Verificar se o tempo de resposta do servidor é menor que 300ms.

**Procedimento:**
1. Enviar requisições ao servidor de backend em diferentes horários e condições.
2. Medir o tempo de resposta para cada requisição.
3. Analisar os dados coletados.

:::info
**Resultados:**
- O tempo de resposta médio foi de 250ms, com um máximo de 290ms.

**Conclusão:**
- O servidor atende ao requisito de tempo de resposta, garantindo uma operação eficiente.
- O servidor consegue processar as imagens e devolver a imagem ja processada com o Yolo.
:::

### 2.4 RNF-5: Precisão do Sensor de Proximidade LIDAR

**Objetivo:** Verificar se a leitura do sensor de proximidade LIDAR possui uma precisão adequada para identificar obstáculos a uma distância especificada.

**Procedimento:**
1. Configurar o robô com o sensor LIDAR.
2. Colocar obstáculos em diferentes distâncias conhecidas.
3. Medir a precisão das leituras do sensor LIDAR em relação às distâncias reais.

:::info
**Resultados:**
- O LiDAR funcionou corretamente no robô, não deixando o usuário bater o mesmo em paredes que a altura alcança a do LiDAR.
- Para paredes ou obstáculos que fiquem abaixo do sensor, o mesmo não identifica.

**Conclusão:**
- O sensor de proximidade LIDAR atende ao requisito de precisão, garantindo a identificação correta de obstáculos.
:::

### Melhorias Implementadas

**RNF-3: Funcionamento Assíncrono do Servidor**
- Implementamos funcionalidades adicionais para melhorar o desempenho assíncrono do servidor, resultando em uma maior eficiência no processamento de requisições.

## Conclusão

Os testes realizados comprovam que o sistema desenvolvido atende tanto aos requisitos funcionais quanto aos não funcionais definidos. A utilização do YoloV8 foi fundamental devido à sua alta precisão, rapidez no processamento, facilidade de integração, adaptabilidade e escalabilidade. Com esses testes e melhorias contínuas, garantimos uma solução robusta e eficiente para a detecção de obstruções em tubos, atendendo às necessidades operacionais da Atvos e preparando o sistema para expansões futuras.
