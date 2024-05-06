---
sidebar_position: 1
custom_edit_url: null
title: "Metodologia"
---
## Introdução

### 1. Levantamento de Requisitos
- Realização de entrevistas com a Atvos para compreender as necessidades e desafios no processo de manutenção dos reboilers.
- Identificação da falta de métricas precisas no processo manual de limpeza e dos riscos associados à alta pressão das mangueiras.

### 2. Definição de Solução
- Brainstorming em equipe para gerar ideias de solução, considerando o uso obrigatório do Turtlebot com câmera.
- Exploração de alternativas, como o uso de placas de LED e algoritmos de visão computacional, levando em conta os requisitos educacionais e de segurança.

### 3. Design e Prototipagem
- Seleção do Turtlebot 3 e Raspberry Pi como componentes principais do sistema.
- Desenvolvimento inicial do software para controle do robô, incluindo a integração com ROS2 e o treinamento do modelo YOLO para detecção de obstruções.
- Planejamento para prototipagem física do robô e testes preliminares.

### 4. Implementação
- Utilização do Raspberry Pi para processamento de dados e comunicação com o Turtlebot.
- Configuração do ambiente de desenvolvimento com ROS2 para permitir a comunicação e controle do robô.
- Desenvolvimento do backend com FastAPI para enviar dados para o banco de dados PostgreSQL.

### 5. Testes e Validação (a ser detalhado)
- Definição de casos de teste para avaliar o desempenho do robô em diferentes cenários.
- Realização de testes de integração e teste de sistema para garantir a funcionalidade e confiabilidade do sistema.
- Validação dos resultados obtidos com a Atvos para garantir que atendem às suas necessidades e expectativas.