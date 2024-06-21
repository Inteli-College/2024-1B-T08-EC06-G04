---
custom_edit_url: null
---

# Requisitos

Nesta documentação, apresentamos uma solução de tecnologia projetada para atender às necessidades específicas e requisitos da Atvos. A eficácia e o sucesso de qualquer solução tecnológica dependem em grande medida da clareza, precisão e compreensão de seus requisitos. Portanto, é fundamental definir e entender adequadamente os requisitos funcionais e não funcionais.

## 1. Requisitos funcionais (RF):

Os requisitos funcionais descrevem as funcionalidades específicas e as operações que a solução deve realizar. Eles são a base para o desenvolvimento e a implementação da solução, delineando as capacidades e os comportamentos esperados do sistema. 

**1.1** RF-1: O robô deve conseguir identificar tubos que estão parcialmente ou totalmente obstruídos através de sua câmera.

**1.2** RF-2: O robô de conseguir fazer a leitura das informações térmicas do reboiler através de um sensor de temperatura.

**1.3** RF-3: O Usuário deve conseguir controlar o robô através de um controle virtual.

**1.4** RF-4: O robô deve conseguir seguir um caminho pré-estipulado pelo usuário.

**1.5** RF-5: O sistema deve retornar dados colhidos assim como uma API.

### Casos de teste 

**RF-1: Identificação de tubos obstruídos**
        
Caso de Teste 1: Posicionar o robô em frente a um tubo parcialmente obstruído. Verificar se o robô identifica corretamente a obstrução.

Caso de Teste 2: Repetir o teste com um tubo totalmente obstruído.

**RF-2: Leitura de informações térmicas**

Caso de Teste 3: Colocar o sensor de temperatura do robô em contato com o reboiler. Verificar se a leitura da temperatura é precisa.

Caso de Teste 4: Variar a temperatura do reboiler e observar se o robô registra as mudanças corretamente.

**RF-3: Controle virtual do robô**

Caso de Teste 5: Simular os comandos de controle (avançar, recuar, girar) usando o controle virtual. Verificar se o robô responde adequadamente.

**RF-4: Navegação pré-estipulada**

Caso de Teste 6: Definir um caminho pré-estipulado para o robô seguir. Verificar se ele segue o caminho corretamente.

**RF-5: Retorno de dados via API**

Caso de Teste 7: Solicitar os dados colhidos pelo robô e verificar se a API retorna as informações corretas.

## 2. Requisitos não funcionais (RNF):

Os requisitos não funcionais referem-se às características e qualidades do sistema que não estão diretamente relacionadas às funcionalidades específicas, mas que são cruciais para garantir seu desempenho, segurança, escalabilidade e usabilidade. Estes incluem aspectos como desempenho, confiabilidade, segurança, usabilidade e manutenibilidade. 

**2.1** RNF-1: A bateria do robô deve durar pelo menos um ciclo de verificação.

**2.2** RNF-2: A latência do controle deve ser menor que 200ms.

**2.3** RNF-3: O servidor deve funcionar de maneira assíncrona.

**2.4** RNF-4: O tempo de resposta do servidor deve ser menor que 300ms.

**2.5** RNF-5: A leitura da temperatura deve possuir uma variação máxima de ±2°C da temperatura real.

### Casos de teste 

**RNF-1: Duração da bateria**

Caso de Teste 1: Executar o robô até que a bateria esteja quase esgotada. Verificar se ela dura pelo menos um ciclo de verificação.

**RNF-2: Latência do controle**

Caso de Teste 2: Enviar comandos de controle e medir o tempo de resposta. Verificar se a latência está dentro do limite especificado.

**RNF-3: Funcionamento assíncrono do servidor**

Caso de Teste 3: Enviar várias solicitações ao servidor simultaneamente e verificar se ele lida com elas de forma assíncrona.

**RNF-4: Tempo de resposta do servidor**

Caso de Teste 4: Enviar uma solicitação ao servidor e medir o tempo de resposta. Verificar se está dentro do limite especificado.

**RNF-5: Leitura de temperatura precisa**

Caso de Teste 5: Variar a temperatura do reboiler e verificar se a leitura do sensor permanece dentro da variação especificada.
