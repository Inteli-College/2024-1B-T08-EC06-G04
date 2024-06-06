---
sidebar_position: 1
custom_edit_url: null
title: "Testes referentes aos RNFs"
---

# Projeto de Detecção de Sujeira em Tubos - Testes de Funcionalidade e Validação de RNFs

## **1. Teste de Funcionalidade (Requisito Funcional)**

### Teste: Controle do Robô (RF-3)

**Objetivo:** Verificar a funcionalidade do controle virtual para operar o robô.

**Procedimento:**
1. Configurar o controle virtual para operar o robô.
2. Iniciar o ciclo de operação do robô através do controle virtual em um ambiente de teste.
3. O robô deve responder aos comandos do controle virtual, se mover conforme direcionado e executar as tarefas designadas.
4. Os usuários devem interagir com o sistema de controle virtual para verificar a eficácia e a responsividade do robô.

:::info
**Usuários Testados:** 4 usuários diferentes foram envolvidos no teste para garantir a consistência e a confiabilidade dos resultados.
:::

**Relatório:**

| Interação | Descrição | Feedback do Usuário | Observações |
|-----------|------------|----------------------|-------------|
| **1: André Leal** | O robô respondeu corretamente aos comandos do controle virtual. Não foram observados atrasos ou erros na execução das tarefas pelo robô. O sistema de controle virtual mostrou-se estável e eficiente, permitindo que o usuário operasse o robô sem dificuldades. | "A experiência de controle é intuitiva e eficiente. A interface do controle virtual é fácil de usar e bastante responsiva. A precisão dos movimentos do robô foi excelente." | O robô funcionou conforme esperado sem a necessidade de ajustes. A interface do controle virtual foi considerada amigável e eficiente. |
| **2: Murilo de Souza Prianti** | Foram notados pequenos atrasos na resposta do robô aos comandos iniciais. Esses atrasos foram causados por uma pequena latência no sistema de comunicação entre o controle virtual e o robô. Após ajustes técnicos no sistema, o robô respondeu de forma adequada na segunda tentativa, mostrando uma melhoria significativa na responsividade. | "Alguns comandos demoraram um pouco para serem executados, mas após ajustes, o desempenho melhorou significativamente. A interface é boa, mas pode ser aprimorada para reduzir a latência." | O relatório sugeriu melhorias na interface do controle virtual para aumentar a responsividade, incluindo otimizações na comunicação entre o controle e o robô. |
| **3: Ólin Medeiros Costa** | O usuário operou o robô sem dificuldades. O robô executou todas as tarefas conforme os comandos recebidos, demonstrando uma boa integração entre o controle virtual e o sistema de navegação do robô. A experiência geral foi positiva, com o robô respondendo de maneira precisa e eficiente. | "Muito fácil de usar e eficiente. O robô respondeu bem a todos os comandos. A interface é intuitiva e não tive problemas durante a operação." | Não foram necessários ajustes adicionais. A interface do controle virtual foi elogiada pela sua facilidade de uso e eficiência. |
| **4: Fernando Vasconcellos** | Sem problemas significativos durante a operação. O robô executou todas as tarefas com precisão. O usuário sugeriu melhorias na interface gráfica para torná-la mais intuitiva e visualmente atraente, facilitando ainda mais a operação do robô. | "A interface poderia ser mais intuitiva e visualmente atraente, mas o robô respondeu perfeitamente aos comandos. A experiência foi positiva, mas há espaço para melhorias estéticas na interface." | O usuário sugeriu melhorias na interface gráfica para uma operação mais intuitiva e visualmente agradável, recomendando um design mais moderno e elementos gráficos mais claros. |

### Conclusões:

O controle virtual demonstrou ser uma ferramenta eficaz para a operação do robô, atendendo às expectativas dos usuários durante os testes. A funcionalidade do controle virtual permitiu uma interação intuitiva e eficiente, sendo fácil de usar e apresentando uma precisão elevada nos comandos. No entanto, foram identificadas algumas áreas que necessitam de melhorias para otimizar a experiência do usuário.

A maioria dos usuários encontrou a interface do controle virtual intuitiva e fácil de usar, destacando a precisão dos movimentos do robô e a facilidade na execução das tarefas. Contudo, alguns usuários relataram pequenos atrasos na resposta do robô aos comandos iniciais, causados por latência no sistema de comunicação. Estes atrasos foram resolvidos após abrir o websocket dentro da máquina local em vez de abrir o mesmo no robô. Todavia dependendo da força do sinal de Wi-Fi, ainda podem ter atrasos referentes a latência.

Os usuários também sugeriram melhorias na interface gráfica do controle virtual. A recomendação é tornar a interface mais intuitiva e visualmente atraente, facilitando ainda mais a operação do robô. As sugestões incluem um design mais moderno e elementos gráficos mais claros, que podem aumentar a satisfação dos usuários e tornar o sistema de controle mais agradável de usar.

Portanto, recomenda-se implementar ajustes para melhorar a responsividade do controle virtual e aprimorar a interface gráfica. Otimizações técnicas na comunicação entre o controle virtual e o robô são essenciais para reduzir latências e garantir uma operação mais fluida. Além disso, melhorias estéticas na interface podem aumentar a satisfação dos usuários, tornando a experiência de uso mais intuitiva e visualmente atraente. Com essas melhorias, espera-se que o sistema de controle virtual se torne ainda mais eficiente e agradável para os usuários, facilitando a operação do robô em diferentes cenários de teste.

## **2. Testes de Validação de Requisitos Não Funcionais (RNFs)**

### 2.1 RNF-1: Duração da Bateria

**Objetivo:** Verificar se a bateria do robô dura pelo menos um ciclo de verificação do reboiler.

**Procedimento:**
1. Carregar a bateria do robô completamente.
2. Iniciar um ciclo completo de verificação em um ambiente de teste.
3. Monitorar a duração da bateria durante todo o ciclo.

**Resultados:**
- A bateria durou aproximadamente 2 horas com o robô em operação contínua, superando o requisito mínimo estabelecido.

**Conclusão:**
- A bateria do robô atende ao requisito de duração, garantindo operação contínua durante um ciclo completo de verificação sem necessidade de recarga. Este resultado assegura a confiabilidade do robô para inspeções prolongadas.

### 2.2 RNF-2: Latência do Controle

**Objetivo:** Verificar se a latência do controle do robô é menor que 200ms.

**Procedimento:**
1. Configurar o sistema de controle do robô.
2. Medir a latência de controle utilizando ferramentas de monitoramento de rede.
3. Repetir o teste em diferentes condições operacionais para verificar a consistência.

**Resultados:**
- A latência média foi de 150ms, com um valor máximo registrado de 180ms, ambos dentro do limite aceitável.

**Conclusão:**
- A latência do controle está bem dentro do limite estabelecido, atendendo ao requisito. Este resultado garante que o robô possa ser controlado com alta precisão e responsividade, essencial para operações em tempo real.

### 2.3 RNF-4: Tempo de Resposta do Servidor

**Objetivo:** Verificar se o tempo de resposta do servidor é menor que 300ms.

**Procedimento:**
1. Enviar requisições ao servidor de backend em diferentes horários e condições.
2. Medir o tempo de resposta para cada requisição.
3. Analisar os dados coletados para avaliar a consistência do desempenho do servidor.

**Resultados:**
- O tempo de resposta médio foi de 250ms, com um máximo de 290ms, ambos abaixo do limite especificado.

**Conclusão:**
- O servidor atende ao requisito de tempo de resposta, garantindo uma operação eficiente. O servidor é capaz de processar as imagens e devolver os resultados processados com o modelo YoloV8 rapidamente, assegurando uma interação fluida e eficiente com o sistema de detecção.

### 2.4 RNF-5: Precisão do Sensor de Proximidade LIDAR

**Objetivo:** Verificar se a leitura do sensor de proximidade LIDAR possui uma precisão adequada para identificar obstáculos a uma distância especificada.

**Procedimento:**
1. Configurar o robô com o sensor LIDAR.
2. Colocar obstáculos em diferentes distâncias conhecidas.
3. Medir a precisão das leituras do sensor LIDAR em relação às distâncias reais.

**Resultados:**
- O sensor LIDAR funcionou corretamente, identificando obstáculos e prevenindo colisões com precisão. No entanto, obstáculos situados abaixo da altura do sensor não foram detectados.

**Conclusão:**
- O sensor de proximidade LIDAR atende ao requisito de precisão para a maioria das situações, garantindo a identificação correta de obstáculos dentro de sua faixa operacional. Para uma detecção mais completa, recomenda-se a consideração de ajustes ou adições de sensores para cobrir pontos cegos.

### Melhorias Implementadas

**RNF-3: Funcionamento Assíncrono do Servidor**
- Foram implementadas funcionalidades adicionais para melhorar o desempenho assíncrono do servidor, resultando em uma maior eficiência no processamento de requisições. Esta melhoria permite que o servidor gerencie múltiplas requisições simultaneamente sem degradação do desempenho.

## **Conclusão**

Os testes realizados comprovam que o sistema desenvolvido atende tanto aos requisitos funcionais quanto aos não funcionais definidos. A utilização do modelo YoloV8 foi fundamental devido à sua alta precisão, rapidez no processamento, facilidade de integração, adaptabilidade e escalabilidade. Com esses testes e melhorias contínuas, garantimos uma solução robusta e eficiente para a detecção de obstruções em tubos, atendendo às necessidades operacionais da Atvos e preparando o sistema para expansões futuras.

O desempenho do robô, aliado à capacidade de resposta do servidor e à precisão dos sensores, assegura que o sistema está bem posicionado para oferecer resultados confiáveis e consistentes em cenários operacionais reais. A continuidade das melhorias, especialmente no que diz respeito à interface de controle e à cobertura de detecção de obstáculos, contribuirá ainda mais para a eficácia e a usabilidade do sistema.
