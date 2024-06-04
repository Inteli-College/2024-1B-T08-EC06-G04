---
sidebar_position: 2
custom_edit_url: null
title: "Backend principal da aplicação para salvar dados no banco de dados"
---

# **Introdução**

Durante a Sprint 4, nossa equipe iniciou o desenvolvimento do backend da aplicação, que desempenhará um papel crucial no armazenamento de imagens dos tubos entupidos após o processamento pela Inteligência Artificial (I.A). A implementação desse backend visa fornecer aos funcionários da Atvos um acesso facilitado a essas imagens, permitindo uma análise precisa para verificar se os tubos realmente estão obstruídos. Esse processo não só melhora a eficiência na identificação de problemas, como também facilita a tomada de decisões rápidas e informadas sobre a necessidade de manutenção.

Além de servir como uma ferramenta de verificação imediata, o backend tem um papel vital no aprimoramento contínuo da I.A. As imagens armazenadas no banco de dados serão usadas para aumentar o conjunto de dados de treinamento da I.A, melhorando sua capacidade de identificar tubos entupidos com maior precisão. Com uma base de dados robusta e diversificada, a I.A poderá aprender com diferentes tipos de obstruções e condições dos tubos, o que resultará em uma maior acurácia e eficiência na detecção de problemas futuros.

O projeto principal envolve o desenvolvimento de um robô especializado em detectar sujeira nos tubos do reboiler. Equipado com uma câmera e algoritmos de I.A., o robô inspeciona os tubos, captura imagens e identifica possíveis obstruções. Essas imagens são então processadas e armazenadas no banco de dados através do backend que estamos desenvolvendo. Esse sistema integrado não apenas permite uma análise detalhada e eficiente por parte dos funcionários, mas também contribui significativamente para o contínuo aprimoramento da tecnologia de I.A., garantindo uma manutenção mais eficaz e uma operação mais suave na planta da Atvos.

## **Tecnologias**

Durante o desenvolvimento do backend, implementamos várias tecnologias avançadas para aprimorar a funcionalidade e a eficiência do nosso projeto. Entre as principais tecnologias utilizadas, destacam-se:

### 1. FastAPI
   - Utilizada para estabelecer a conexão com a base de dados e fornecer uma interface eficiente e robusta para a comunicação.
   - Escolhemos FastAPI devido ao seu alto desempenho e facilidade na criação de APIs rápidas e seguras.

### 2. TinyDB
   - Implementada como a base de dados para armazenar as imagens e metadados associados aos tubos entupidos.
   - TinyDB é uma solução leve e simples, ideal para a manipulação de dados e integração direta com a FastAPI.

Essas tecnologias foram fundamentais para garantir a eficiência na comunicação entre o backend e a base de dados, além de proporcionar uma solução de armazenamento de dados leve e eficaz.

:::info
As tecnologias referentes a outros serviços estarão em suas respectivas partes desta documentação.
:::

## **Desenvolvimento do Backend**

Para o desenvolvimento do backend, utilizamos tecnologias avançadas para garantir uma solução robusta e eficiente. O processo de criação envolveu várias etapas importantes:

### 1. Configuração do Ambiente de Desenvolvimento
   - Inicialmente, configuramos o ambiente de desenvolvimento utilizando ferramentas modernas para assegurar uma estrutura básica e eficiente.
   - Instalamos todas as dependências essenciais, incluindo bibliotecas adicionais que facilitam o desenvolvimento e a integração com outras tecnologias.

### 2. Utilização do FastAPI
   - Escolhemos FastAPI para estabelecer a conexão com a base de dados e fornecer uma interface eficiente e robusta para a comunicação.
   - A FastAPI foi configurada para criar APIs rápidas, seguras e de alto desempenho, facilitando a integração com outros componentes do sistema.

### 3. Implementação do TinyDB
   - Optamos pelo TinyDB como a base de dados para armazenar as imagens e metadados associados aos tubos entupidos.
   - O TinyDB, sendo uma solução leve e simples, foi ideal para a manipulação de dados e integração direta com a FastAPI, garantindo eficiência no armazenamento e recuperação de dados.

### 4. Estruturação do Projeto
   - Organizamos o backend em módulos bem definidos, garantindo que cada parte do sistema fosse independente e reutilizável.
   - Essa modularidade facilita a manutenção do código e a implementação de novas funcionalidades no futuro.

### 5. Desenvolvimento de Funcionalidades Críticas
   - Implementamos funcionalidades essenciais para o armazenamento seguro e eficiente das imagens e metadados dos tubos entupidos.
   - Desenvolvemos mecanismos para garantir a integridade dos dados e a eficiência na recuperação das informações armazenadas.

A criação do backend com FastAPI e TinyDB resultou em uma solução robusta e eficiente, capaz de atender às necessidades específicas do projeto e proporcionar uma base sólida para o desenvolvimento de funcionalidades adicionais no futuro.

:::info
O projeto foi criado levando em consideração a **escalabilidade** do código, permitindo fácil manutenção e expansão.
:::