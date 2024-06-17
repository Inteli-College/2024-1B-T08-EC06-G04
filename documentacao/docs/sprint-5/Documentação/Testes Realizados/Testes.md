---
sidebar_position: 2
custom_edit_url: null
title: "Testes"
---

# Introdução

&emsp;&emsp; Ao longo das 5 Sprints, foram feitos uma série de testes afim de garantir o funcionamento do projeto em cada uma das 5 Sprints, os mesmos serviram para nos auxiliar e nos guiar sobre o que deveria ser mudado e o que estava funcionando corretamente. Além de nos dar uma visão geral do modo de operação do usuário final.

&emsp;&emsp; Os testes foram realizados em ambientes controlados, simulando o ambiente real de operação do robô, com o objetivo de garantir que o robô fosse capaz de realizar as tarefas propostas.

&emsp;&emsp; Segue abaixo os testes que foram realizados e documentados na Sprint 4

### Teste: Controle do Robô (RF-3)

**Objetivo:** Verificar a funcionalidade do controle virtual para operar o robô de forma eficaz e segura, garantindo que todas as funcionalidades estejam operacionais e que o sistema responda adequadamente aos comandos dos usuários.

**Roteiro:**
1. **Configuração Inicial:**
   - Configurar o controle virtual para operar o robô, assegurando que todas as conexões estejam estáveis e que o sistema esteja pronto para receber comandos.
   - Conectar o robô aos usuários, permitindo que eles tenham acesso ao controle virtual e possam interagir diretamente com o robô.

2. **Operação Básica:**
   - Orientar os usuários a moverem o robô utilizando o controle virtual, verificando se o robô responde corretamente aos comandos de movimentação.
   - Comandar o robô para parar completamente, certificando-se de que o robô obedece imediatamente ao comando de parada.
   - Comandar o robô para girar, avaliando se a rotação é precisa e corresponde aos comandos dados pelos usuários.

3. **Interação com o Ambiente:**
   - Conduzir o robô até um "cano" específico localizado no ambiente de teste, utilizando o controle virtual para guiar o robô até o ponto desejado.
   - Capturar uma imagem do "cano" utilizando a câmera embutida no robô, assegurando que a qualidade da imagem seja suficiente para análise posterior.
   - Processar a imagem capturada utilizando a I.A., para interpretar o conteúdo da imagem e fornecer feedback ou ações baseadas na análise realizada.

4. **Interrupção e Reinício:**
   - Tentar colidir o robô de frente contra uma parede, observando como o sistema de controle reage e se o robô para imediatamente ao detectar a colisão.
   - Tentar colidir o robô de ré contra uma parede, verificando novamente a resposta do sistema de controle e a eficiência dos sensores de detecção.
   - Acionar o botão de parada de emergência para interromper todas as operações do robô, certificando-se de que o sistema de segurança é acionado corretamente e todas as atividades do robô são cessadas imediatamente.
   - Reiniciar a aplicação após a ativação da parada de emergência, verificando se o robô retorna ao estado normal de operação e pode ser controlado novamente sem problemas.

**Resultados Esperados:**
- **Conexão e Controle:**
  - O controle virtual deve se conectar corretamente ao robô e permitir que os usuários interajam sem interrupções, demonstrando uma conexão estável e confiável.
  - O robô deve responder de forma precisa e imediata aos comandos de movimento, parada e rotação dados pelos usuários através do controle virtual, indicando uma alta responsividade do sistema.

- **Processamento de Imagens:**
  - A captura e o processamento da imagem do "cano" devem ser realizados com sucesso, com a I.A. fornecendo uma análise precisa do conteúdo da imagem e ações ou feedback adequados baseados nessa análise.
  - Qualquer popup gerado pelo sistema deve ser fechado conforme solicitado pelos usuários, garantindo que a interface do controle virtual seja limpa e livre de obstruções desnecessárias.

- **Segurança e Recuperação:**
  - As tentativas de colisão devem ser registradas e o robô deve parar conforme esperado ao detectar uma possível colisão, demonstrando a eficácia dos sensores de segurança.
  - A parada de emergência deve interromper imediatamente todas as operações do robô, assegurando que o sistema de segurança funcione corretamente e proteja o robô e o ambiente.
  - Após a ativação da parada de emergência, o robô deve ser capaz de reiniciar normalmente e retornar ao estado operacional sem problemas, mostrando a resiliência do sistema a interrupções emergenciais.


:::info
**Usuários Testados:** 4 usuários diferentes foram envolvidos no teste para garantir a consistência e a confiabilidade dos resultados.
:::

**Relatório:**

| Interação | Descrição | Feedback do Usuário | Observações |
|-----------|------------|----------------------|-------------|
| **1: André Leal** | O robô respondeu corretamente aos comandos do controle virtual. Não foram observados atrasos ou erros na execução das tarefas pelo robô. O sistema de controle virtual mostrou-se estável e eficiente, permitindo que o usuário operasse o robô sem dificuldades. | "A experiência de controle é intuitiva e eficiente. A interface do controle virtual é fácil de usar e bastante responsiva. A precisão dos movimentos do robô foi excelente. Faltou feedback a respeito do LiDAR (Por que o robô não mexia para trás ou frente em algumas horas)" | O robô funcionou conforme esperado sem a necessidade de ajustes. A interface do controle virtual foi considerada amigável e eficiente. |
| **2: Esther Hikari** | Foram notados pequenos atrasos na resposta do robô aos comandos iniciais. Esses atrasos foram causados por uma pequena latência no sistema de comunicação entre o controle virtual e o robô ou por causa da rede de Internet do Inteli no momento dos testes. | "Achou muito interessante a interface, e gostou muito da I.A ja embarcada no robô, conseguiu mexer o mesmo e fazer as leituras com a câmera." | Teve certos problemas para saber por que o robô não mexia para frente ou trás em alguns momentos (Por causa do LiDAR não deixando graças a uma parede) porém logo após alguns segundos percebeu e seguiu sem problemas. |
| **3: Enya Oliveria** | Ela conseguiu operar o robô sem dificuldades. O robô executou todas as tarefas conforme os comandos recebidos, demonstrando uma boa integração entre o controle virtual e o sistema de navegação do robô. A experiência geral foi positiva, com o robô respondendo de maneira precisa e eficiente. | "Muito fácil de usar e eficiente. O robô respondeu bem a todos os comandos. A interface é intuitiva, simples e bonita e não tive problemas durante a operação. Gostei bastante dos ícones e dos botões" | Não foram necessários ajustes adicionais. A interface do controle virtual foi elogiada pela sua facilidade de uso e eficiência. |
| **4: Fernando Machado** | Sem problemas significativos durante a operação. O robô executou todas as tarefas com precisão. O usuário sugeriu melhorias na resposividade do robô pois o mesmo no momento de teste estava com uma latência superior a 1000ms (1s) todavia possivelmente por causa da Internet do Inteli, dado que após 10 minutos ele voltou ao normal. | "O robô poderia responder aos comandos um pouco mais rapidamente, porém de resto achei muito intuitiva e boa interface, além de ter gostado muito do sistema da Inteligência Artificial." | Sugeriu algumas melhorias a respeito da responsividade, porém todas ja foram aplicadas anteriormente e não há muitas melhorias extras a se fazer. Afim de ter o melhor potencial do robô é essencial estar em uma boa rede de Internet. |

### Conclusões:

O controle virtual demonstrou ser uma ferramenta eficaz para a operação do robô, atendendo às expectativas dos usuários durante os testes. A funcionalidade do controle virtual permitiu uma interação intuitiva e eficiente, sendo fácil de usar e apresentando uma precisão elevada nos comandos. No entanto, foram identificadas algumas áreas que necessitam de melhorias para otimizar a experiência do usuário.

**Principais pontos destacados nos relatórios:**

1. **Interface Intuitiva e Eficiente:**
   - A maioria dos usuários encontrou a interface do controle virtual intuitiva e fácil de usar. Eles destacaram a precisão dos movimentos do robô e a facilidade na execução das tarefas. Isso indica que o design da interface e a disposição dos controles são adequados para uma operação simples e eficaz.
   - O feedback positivo sobre a interface gráfica incluiu elogios à sua aparência amigável e eficiente. Os ícones e botões utilizados foram bem recebidos, sugerindo que eles são claros e funcionais, contribuindo para uma experiência de usuário agradável.

2. **Desempenho do Robô:**
   - O robô funcionou conforme esperado durante os testes, demonstrando uma boa integração entre o controle virtual e o sistema de navegação. Isso mostra que a comunicação entre o software de controle e o hardware do robô é robusta e confiável.
   - Os usuários relataram experiências positivas quanto à resposta e precisão dos comandos, o que reforça a eficácia do sistema em permitir que o robô execute as tarefas com precisão.

3. **Latência e Conectividade:**
   - Alguns usuários relataram pequenos atrasos na resposta do robô aos comandos iniciais. Esses atrasos foram atribuídos a problemas de latência na comunicação entre o controle virtual e o robô, possivelmente devido à qualidade da rede de Internet do Inteli no momento dos testes.
   - Foi observado que a qualidade da conexão de Internet impacta significativamente a responsividade do robô. Isso sugere que a infraestrutura de rede desempenha um papel crucial na performance do sistema e na experiência do usuário.

4. **Feedback sobre LiDAR:**
   - Houve comentários sobre a falta de feedback do LiDAR, especialmente em situações onde o robô não se movia para frente ou para trás devido à detecção de obstáculos. Essa falta de feedback pode causar confusão para o usuário, que pode não entender por que o robô não está respondendo como esperado.

**Recomendações:**

1. **Melhoria na Responsividade:**
   - É essencial realizar otimizações técnicas na comunicação entre o controle virtual e o robô para reduzir latências, garantindo uma operação mais fluida e responsiva. Isso pode incluir melhorias nos protocolos de comunicação e na infraestrutura de rede utilizada durante os testes.
   - Verificar e aprimorar a infraestrutura de rede durante os testes é fundamental para minimizar problemas de conectividade. Isso pode envolver o uso de redes de alta velocidade e a redução de interferências que possam causar atrasos na comunicação.

2. **Feedback do LiDAR:**
   - Implementar um sistema de feedback visual ou auditivo que informe o usuário sobre a detecção de obstáculos pelo LiDAR pode melhorar significativamente a experiência de uso. Isso ajudará o usuário a entender por que o robô não está se movendo em certas direções e a tomar decisões informadas durante a operação.

3. **Aprimoramentos na Interface Gráfica:**
   - Tornar a interface mais intuitiva e visualmente atraente pode facilitar ainda mais a operação do robô. Isso pode incluir ajustes no design dos ícones, botões e outros elementos visuais para torná-los mais claros e agradáveis.
   - Considerar um design mais moderno e elementos gráficos mais claros pode aumentar a satisfação dos usuários. Melhorias estéticas na interface podem tornar o sistema de controle mais agradável de usar, aumentando a eficiência e a satisfação geral.

Com essas melhorias, espera-se que o sistema de controle virtual se torne ainda mais eficiente e agradável para os usuários, facilitando a operação do robô em diferentes cenários de teste. Essas mudanças não só melhorarão a funcionalidade técnica do sistema, mas também a experiência geral do usuário, tornando o controle do robô uma tarefa mais intuitiva e satisfatória.


## **2.0 Testes referentes aos RNF**

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
Para melhorar o desempenho assíncrono do servidor, implementamos várias funcionalidades adicionais utilizando o framework FastAPI. O FastAPI permite que as operações sejam processadas de maneira assíncrona, o que é crucial para lidar com múltiplas requisições simultâneas sem comprometer a performance do sistema. A escolha do FastAPI se deve à sua capacidade de simplificar a implementação de operações assíncronas com o uso de async/await, proporcionando uma base robusta para o desenvolvimento de aplicações de alta performance.

Uma das principais melhorias foi a adoção do modelo de programação assíncrona, utilizando as palavras-chave async e await do Python. Isso permite que o servidor execute operações de entrada e saída (I/O) de forma não bloqueante, otimizando o uso de recursos e melhorando a capacidade de resposta do sistema. Com isso, o servidor consegue gerenciar várias requisições ao mesmo tempo, evitando a degradação do desempenho mesmo em situações de alta demanda.

Além disso, implementamos pooling de conexões para gerenciar as conexões ao banco de dados de maneira mais eficiente. O pooling permite a reutilização de conexões já estabelecidas, reduzindo a latência e o overhead associado à criação de novas conexões para cada requisição. Essa abordagem, combinada com a utilização de middlewares otimizados para logging e monitoramento, assegura uma operação mais fluida e uma rápida identificação de possíveis gargalos no processamento das requisições. Essas melhorias resultaram em um servidor mais eficiente e responsivo, capaz de atender a um grande volume de tráfego com alta performance.


## **Conclusão**

Analisando estes testes aós a finalização completa tanto do código do projeto, quanto da parte física do robô, conseguimos concluir que o projeto está em um estado avançado e pronto para ser entregue. Os testes realizados garantiram que o robô é capaz de operar de forma eficaz e segura, atendendo aos requisitos funcionais e não funcionais estabelecidos. A integração entre o controle virtual e o robô foi bem-sucedida, permitindo uma operação intuitiva e eficiente. A duração da bateria, a latência do controle, o tempo de resposta do servidor e a precisão do sensor LIDAR foram avaliados e atenderam aos requisitos estabelecidos, demonstrando a robustez e confiabilidade do sistema.

Além disso, as melhorias implementadas no servidor, como o funcionamento assíncrono e o pooling de conexões, contribuíram para aprimorar o desempenho e a eficiência do sistema, garantindo uma operação mais fluida e responsiva. Com essas melhorias, o sistema está mais preparado para lidar com situações de alta demanda e oferecer uma experiência de usuário aprimorada.

Por fim o projeto está pronto para ser entregue e utilizado em cenários reais de inspeção e monitoramento, proporcionando uma solução inovadora e eficaz para a detecção de vazamentos em reboilers. Agradecemos a todos os envolvidos no desenvolvimento e teste do projeto, e esperamos que ele possa contribuir significativamente para a segurança e eficiência das operações industriais.