# Tacomaré

<p align="center">
<a href= "https://www.inteli.edu.br/"> <img src="https://github.com/Inteli-College/2024-T0008-EC05-G03/assets/85657433/afc793e7-2a20-4207-8832-0c998187c537" alt="Inteli - Instituto de Tecnologia e Liderança" border="0"></a>
</p>


## Integrantes

<div align="center">
  <table>
    <tr>
        <td align="center"><a href="https://www.linkedin.com/in/antonio-guimar%C3%A3es-2bb961264/?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=ios_app"><img style="border-radius: 50%;" src="https://media.licdn.com/dms/image/D4D03AQHWhBcP5iT58g/profile-displayphoto-shrink_200_200/0/1703212257337?e=1719446400&v=beta&t=-hxRCaieslIx5EnUF1kdL05ndTW3xRoNzSNYov5bHVI" width="100px;" alt=""/><br><sub><b>Antônio Guim.</b></sub></a></td>
        <td align="center"><a href="https://www.linkedin.com/in/breno-santos-0843131b8/"><img style="border-radius: 50%;" src="https://media.licdn.com/dms/image/D4D03AQGrZG9Tv9JdZg/profile-displayphoto-shrink_200_200/0/1711976052733?e=1719446400&v=beta&t=EnMiEdG5b93zUKIq8OwufXXMMxv9TYyaJLjetfNcPYI" width="100px;" alt=""/><br><sub><b>Breno Santos</b></sub></a></td>
        <td align="center"><a href="https://www.linkedin.com/in/bruno-gottardo-conti-a9625726a/"><img style="border-radius: 40%;" src="https://media.licdn.com/dms/image/D5603AQHP0Vtxfm1MWA/profile-displayphoto-shrink_200_200/0/1678741883613?e=1719446400&v=beta&t=aI8huLHMqpXm3Kc_CJB2WhAftYQ3pQNLU2JtiRkeHWw" width="100px;" alt=""/><br><sub><b>Bruno Conti</b></sub></a></td>
        <td align="center"><a href="https://www.linkedin.com/in/gabrielle-mitoso/"><img style="border-radius: 50%;" src="https://media.licdn.com/dms/image/D4D03AQHs0Kfrwmc5gA/profile-displayphoto-shrink_200_200/0/1676560341538?e=1719446400&v=beta&t=UxXb2ubre3WZkvImu-8SxpZJBlLY1FXIoPsMXy23MZk" width="100px;" alt=""/><br><sub><b>Gabrielle Mit.</b></sub></a></td>
        <td align="center"><a href="https://www.linkedin.com/in/gabriel-gallo-m-coutinho-443809232/"><img width="100px;" style="border-radius: 50%;" src="https://media.licdn.com/dms/image/D4E03AQGQ_hxvNv8a2w/profile-displayphoto-shrink_200_200/0/1665073284301?e=1719446400&v=beta&t=wNURhSaL_KL_5AYa545NKxdEG3-yDKq-4aRb-1JFD90" width="100px;" alt=""/><br><sub><b>Gabriel Gallo</b></sub></a></td>
        <td align="center"><a href="https://www.linkedin.com/in/luan-ramos-de-mello-253b28268/"><img style="border-radius: 50%;" src="https://media.licdn.com/dms/image/D4D03AQF5k4FEfaI4mg/profile-displayphoto-shrink_200_200/0/1698150342373?e=1719446400&v=beta&t=cZXl3FXyEFyfAG8cmRZykVDRtpGO6uRIu2l0UGGtFnQ" width="100px;" alt=""/><br><sub><b>Luan Ramos</b></sub></a></td>
        <td align="center"><a href="https://www.linkedin.com/in/rafaelarojas/"><img style="border-radius: 50%;" src="https://media.licdn.com/dms/image/D4D03AQEZUiXhFkM1SQ/profile-displayphoto-shrink_200_200/0/1685488274068?e=1719446400&v=beta&t=74cw_CRyN__dpurQW6PIAAjCxi3boMq-l_xdUiOVM8o" width="100px;" alt=""/><br><sub><b>Rafaela Rojas</b></sub></a></td>
    </tr>
  </table>
</div>

## Descrição 

📜| Este projeto desenvolvido para a Atvos utiliza inteligência artificial (IA) e um robô turtlebot para realizar a inspeção dos tubos dos reboilers na produção de açúcar de cana. Através de uma abordagem dupla, nossa solução busca identificar obstruções que possam comprometer a eficiência do processo.

Uma das soluções envolve o uso de sensores de temperatura acoplados ao turtlebot para monitorar e registrar a temperatura dos tubos. Anomalias na temperatura podem indicar a presença de entupimentos, possibilitando a criação de um mapa de calor para identificar os canos entupidos.

A outra solução, uma câmera endoscópica percorre o interior dos tubos. Esta câmera, equipada com o modelo de IA YOLO, analisa as imagens capturadas para detectar visualmente quaisquer sinais de entupimento. A combinação destas tecnologias permite uma avaliação abrangente e precisa do estado dos tubos, garantindo uma operação mais estável e eficiente.


---


##  Estrutura de pastas

```
└── documentacao
│   ├── blog
│   ├── docs
|   │   ├── sprint-1
|   │   ├── sprint-2
|   │   ├── sprint-3 
|   │   ├── sprint-4 
|   │   └── sprint-5
│   ├── src
|   │   ├── components
|   │   ├── css
|   │   └── pages
│   └── static
└──
```

---

## Inicialização

Para inicializar a interface CLI, se seu terminal for .bash, execute o seguinte no seu terminal na respectiva ordem:

```bash
cd 2024-1B-T08-EC06-G04

chmod +x exec_bash.sh

./exec_bash.sh
```

Já, se seu terminal for .zsh, execute os seguinte códigos no terminal:

```bash
cd 2024-1B-T08-EC06-G04

chmod +x exec_zsh.sh

./exec_zsh.sh
```

### Workspace ROS

---

## Histórico de lançamentos

- 0.1.0 - 16/02/2024
  - Entendimento do negócio
  - Arquitetura do sistema
  - Metadesign
  - Identidade visual do grupo

#  Licença/License

<img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/cc.svg?ref=chooser-v1"><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/by.svg?ref=chooser-v1"><p xmlns:cc="http://creativecommons.org/ns#" xmlns:dct="http://purl.org/dc/terms/"><a property="dct:title" rel="cc:attributionURL" href="https://github.com/Inteli-College/2024-T0008-EC05-G03">GRUPO 4</a> by <a rel="cc:attributionURL dct:creator" property="cc:attributionName">Inteli, - <a href="https://www.linkedin.com/in/antonio-guimar%C3%A3es-2bb961264/?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=ios_app">Antonio Guimarães</a>, <a href="https://www.linkedin.com/in/breno-santos-0843131b8/">Breno Santos</a>, <a href="https://www.linkedin.com/in/bruno-gottardo-conti-a9625726a/">Bruno Conti</a>, <a href="https://www.linkedin.com/in/gabrielle-mitoso/">Gabrielle Mitoso</a>, <a href="https://www.linkedin.com/in/gabriel-gallo-m-coutinho-443809232/">Gabriel Gallo</a>, <a href="https://www.linkedin.com/in/luan-ramos-de-mello-253b28268/">Luan Ramos</a> <a href="https://www.linkedin.com/in/rafaelarojas/">Rafaela Rojas</a>, </a> is licensed under <a href="http://creativecommons.org/licenses/by/4.0/?ref=chooser-v1" target="_blank" rel="license noopener noreferrer" style="display:inline-block;">Attribution 4.0 International</a>.</p>
