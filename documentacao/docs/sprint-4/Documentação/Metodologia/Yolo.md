---
sidebar_position: 3
custom_edit_url: null
title: "Inteligência artificial com o modelo YOLO para detectar canos entupidos"
---

# Introdução

Durante a Sprint 4, nossa equipe iniciou o desenvolvimento do backend da aplicação, que desempenhará um papel crucial no armazenamento de imagens dos tubos entupidos após o processamento pela Inteligência Artificial (I.A). A implementação desse backend visa fornecer aos funcionários da Atvos um acesso facilitado a essas imagens, permitindo uma análise precisa para verificar se os tubos realmente estão obstruídos. Esse processo não só melhora a eficiência na identificação de problemas, como também facilita a tomada de decisões rápidas e informadas sobre a necessidade de manutenção.

Além de servir como uma ferramenta de verificação imediata, o backend tem um papel vital no aprimoramento contínuo da I.A. As imagens armazenadas no banco de dados serão usadas para aumentar o conjunto de dados de treinamento da I.A, melhorando sua capacidade de identificar tubos entupidos com maior precisão. Com uma base de dados robusta e diversificada, a I.A poderá aprender com diferentes tipos de obstruções e condições dos tubos, o que resultará em uma maior acurácia e eficiência na detecção de problemas futuros.

O projeto principal envolve o desenvolvimento de um robô especializado em detectar sujeira nos tubos do reboiler. Equipado com uma câmera e algoritmos de I.A., o robô inspeciona os tubos, captura imagens e identifica possíveis obstruções. Essas imagens são então processadas e armazenadas no banco de dados através do backend que estamos desenvolvendo. Esse sistema integrado não apenas permite uma análise detalhada e eficiente por parte dos funcionários, mas também contribui significativamente para o contínuo aprimoramento da tecnologia de I.A., garantindo uma manutenção mais eficaz e uma operação mais suave na planta da Atvos.

Para o modelo YoloV8, utilizamos um dataset de nossa autoria, desenvolvido especificamente para abordar melhor o problema de detecção de sujeira nos tubos. Embora o YoloV8 não tenha sido criado por nossa equipe, ele foi escolhido por sua eficácia e precisão na detecção de objetos. O uso desse modelo, combinado com nosso dataset personalizado, permite uma identificação mais precisa e confiável das obstruções, refletindo as condições reais encontradas na planta da Atvos.


## **Tecnologias**

Afim de que a identificação de tubos sujos nas imagens recebidas pelo robô, implementamos o modelo YoloV8, com nosso dataset de autoria própria afim de conseguir com êxito identificar os tubos. A principal tecnologia utilizada foi a biblioteca Yolo do Python.

:::warning
Python tem de estar na versão **3.11**, qualquer outra versão terá problemas no **TREINAMENTO** apenas
:::

### 1. YoloV8

    - Utilizado para a detecção de sujeira nos tubos com base nas imagens capturadas pelo robô.
    - Escolhemos YoloV8 devido à sua eficácia e precisão na detecção de objetos, características essenciais para nosso objetivo de identificar obstruções nos tubos do reboiler.
    - O YoloV8 oferece uma alta taxa de acerto e rapidez no processamento de imagens, o que é crucial para a operação em tempo real necessária na planta da Atvos.
    - Utilizamos um dataset de nossa autoria para treinar o YoloV8, garantindo que ele seja ajustado especificamente para o problema de detecção de sujeira nos tubos. Isso nos permite obter um modelo altamente especializado e eficaz nas condições reais da planta.
    - A arquitetura avançada do YoloV8 proporciona uma balanceada combinação entre precisão e velocidade, fatores críticos para aplicações industriais onde a detecção rápida e precisa pode evitar paralisações e aumentar a eficiência operacional.

### 2. Roboflow

	- Utilizado para a preparação e gerenciamento do dataset, facilitando a anotação e organização das imagens de treino.
	- Escolhemos o Roboflow por sua interface intuitiva e suas poderosas ferramentas de pré-processamento de dados, que nos permitiram criar um dataset de alta qualidade para treinar o modelo YoloV8.
	- A plataforma Roboflow também oferece suporte para a exportação de datasets em vários formatos, compatíveis com diferentes frameworks de machine learning, incluindo o YoloV8.

Em relação a parte da I.A, foi apenas usado o Yolo para treinamento e o Roboflow para fazer o dataset de forma que o Yolo consiga ser treinado.

## **Utilização do YoloV8 e Roboflow**

Para a implementação da detecção de sujeira nos tubos, utilizamos tecnologias avançadas como o YoloV8 e o Roboflow. O processo de criação envolveu várias etapas importantes:

### 1. Preparação do Dataset com Roboflow
   - Inicialmente, utilizamos o Roboflow para preparar e gerenciar nosso dataset, facilitando a anotação e organização das imagens de treino.
   - Escolhemos o Roboflow por sua interface intuitiva e suas poderosas ferramentas de pré-processamento de dados, que nos permitiram criar um dataset de alta qualidade.
   - A plataforma Roboflow oferece suporte para a exportação de datasets em vários formatos, compatíveis com diferentes frameworks de machine learning, incluindo o YoloV8.

:::info
O dataset que foi utilizado está disponível [aqui](https://universe.roboflow.com/project-y7tgq/tacomare), sinta-se livre para utilizá-lo de acordo com suas necessidades.
:::

### 2. Escolha e Configuração do YoloV8
   - Escolhemos o YoloV8 devido à sua eficácia e precisão na detecção de objetos, características essenciais para nosso objetivo de identificar obstruções nos tubos do reboiler.
   - O YoloV8 oferece uma alta taxa de acerto e rapidez no processamento de imagens, o que é crucial para a operação em tempo real necessária na planta da Atvos.
   - Configuramos o YoloV8 para integrar-se perfeitamente com nosso pipeline de dados, garantindo que as imagens capturadas pelo robô fossem processadas de maneira eficiente.

### 3. Treinamento do Modelo
   - Utilizamos um dataset de nossa autoria para treinar o YoloV8, garantindo que ele fosse ajustado especificamente para o problema de detecção de sujeira nos tubos.
   - O treinamento foi realizado com um foco na maximização da precisão e eficiência do modelo, considerando as condições reais encontradas na planta da Atvos.
   - A arquitetura avançada do YoloV8 proporcionou uma balanceada combinação entre precisão e velocidade, fatores críticos para aplicações industriais onde a detecção rápida e precisa pode evitar paralisações e aumentar a eficiência operacional.

:::info
Atualmente estamos utilizando um modelo de tamanho "L" (large), de acordo com o projeto, o tamanho do mesmo **pode** mudar entre os disponíveis `(N, S, L, X)`
:::

### 4. Validação e Testes
   - Realizamos extensivos testes para validar a precisão e a robustez do modelo YoloV8.
   - Avaliamos o desempenho do modelo em condições variadas para garantir sua confiabilidade e eficiência em diferentes cenários de obstrução dos tubos.

### 5. Integração com o Sistema
   - Integrado com o backend desenvolvido, o YoloV8 processa as imagens capturadas pelo robô e identifica possíveis obstruções.
   - As imagens processadas e os resultados das detecções são então armazenados no banco de dados, permitindo uma análise detalhada e eficiente por parte dos funcionários.
   - O Yolo apenas processa as imagens quando um botão no Frontend é pressionado, afim de evitar sobrecarregamento do sistema e possíveis `crashes`

O uso combinado do YoloV8 e Roboflow resultou em uma solução robusta e eficiente, capaz de identificar com precisão as obstruções nos tubos, proporcionando uma base sólida para o aprimoramento contínuo da tecnologia de I.A.

## **Bibliotecas utilizadas para treinamento**

Para o desenvolvimento e validação do modelo YoloV8, utilizamos um conjunto específico de bibliotecas e métodos que foram essenciais para garantir a precisão e eficiência do modelo. As principais bibliotecas utilizadas foram:

### 1. Ultralytics
   - Utilizada para acessar e implementar o modelo YoloV8.
   - Oferece uma interface simples e eficiente para treinar, validar e inferir com o modelo YoloV8.
   - Facilita o processo de configuração e ajuste dos parâmetros do modelo, garantindo um desempenho otimizado.

### 2. Torch
   - Biblioteca fundamental para a criação e treinamento de modelos de deep learning.
   - Proporciona funcionalidades avançadas para operações tensoriais e algoritmos de aprendizado de máquina.
   - Essencial para o backend do YoloV8, permitindo o treinamento eficiente e a inferência rápida.

Essas bibliotecas foram escolhidas por suas funcionalidades avançadas e sua capacidade de integrar-se perfeitamente com o nosso pipeline de desenvolvimento, garantindo uma solução robusta e eficiente para a detecção de obstruções nos tubos.