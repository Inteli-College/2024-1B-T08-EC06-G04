---
custom_edit_url: null
---

# Requisitos

Nesta documentação, apresentamos uma solução de tecnologia projetada para atender às necessidades específicas e requisitos da Atvos. A eficácia e o sucesso de qualquer solução tecnológica dependem em grande medida da clareza, precisão e compreensão de seus requisitos. Portanto, é fundamental definir e entender adequadamente os requisitos funcionais e não funcionais.

## 1. Requisitos funcionais (RF):

Os requisitos funcionais descrevem as funcionalidades específicas e as operações que a solução deve realizar. Eles são a base para o desenvolvimento e a implementação da solução, delineando as capacidades e os comportamentos esperados do sistema. 

**1.1** RF-1: O robo deve conseguir identificar tubos que estão parcialmente ou totalmente obstruidos atráves de sua camêra.

**1.2** RF-2: O robo de conseguir fazer a leitura das informações termicas do reboiler através de um sensor de temperatura.

**1.3** RF-3: O Usuário deve conseguir controlar o robo através de um controle virtual.

**1.4** RF-4: O robo deve conseguir seguir um caminho pré-estipulado pelo usuário.

**1.5** RF-5: O sistema deve retornar dados colhidos assim como uma API.

## 2. Requisitos não funcionais (RNF):

Os requisitos não funcionais referem-se às características e qualidades do sistema que não estão diretamente relacionadas às funcionalidades específicas, mas que são cruciais para garantir seu desempenho, segurança, escalabilidade e usabilidade. Estes incluem aspectos como desempenho, confiabilidade, segurança, usabilidade e manutenibilidade. 

**2.1** RNF-1: A bateria do robô deve durar pelo menos um ciclo de verificação.

**2.2** RNF-2: A latência do controle deve ser menor que 200ms.

**2.3** RNF-3: O servidor deve funcionar de maneira assíncrona.

**2.4** RNF-4: O tempo de resposta do servidor deve ser menor que 300ms.

**2.5** RNF-5: A leitura da temperatura deve possuir uma variação máxima de ±1°C da temperatura real.