#ifndef HW_CONFIG_H
#define HW_CONFIG_H


/* Timer and PWM */
#define TIM_PWM			htim1				// PWM/ISR timer handle
#define TIM_CH_U		TIM_CHANNEL_1		// Terminal U timer channel
#define TIM_CH_V		TIM_CHANNEL_2		// Terminal V timer channel
#define TIM_CH_W		TIM_CHANNEL_3		// Terminal W timer channel
#define INVERT_DTC		1					// PWM inverting (1) or non-inverting (0)

#define TIMx TIM1
#define TIMx_CLK_ENABLE()  __HAL_RCC_TIM1_CLK_ENABLE()
#define TIMx_CLK_DISABLE()  __HAL_RCC_TIM1_CLK_DISABLE()
#define TIMx_GPIO_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define TIMx_PORT GPIOE
#define TIMx_GPIO_AF GPIO_AF1_TIM1
#define TIM_CH_U_Pin GPIO_PIN_8
#define TIM_CH_V_Pin GPIO_PIN_11
#define TIM_CH_W_Pin GPIO_PIN_13

/* ISRs */
#define PWM_ISR			TIM1_UP_TIM10_IRQn	// PWM Timer ISR
#define CAN_ISR			CAN1_RX0_IRQn		// CAN Receive ISR

/* ADC */

#define ADC_CH_MAIN		hadc1				// ADC channel handle which drives simultaneous mode
#define ADC_CH_IA		hadc1					// Phase A current sense ADC channel handle.  0 = unused
#define ADC_CH_IB		hadc2				// Phase B current sense ADC channel handle.  0 = unused
#define ADC_CH_IC		0				// Phase C current sense ADC channel handle.  0 = unused
#define ADC_CH_VBUS		hadc3				// Bus voltage ADC channel handle.  0 = unused

#define ADC1_CHANNEL       ADC_CHANNEL_10
#define ADC1_Pin           GPIO_PIN_0
#define ADC1_Port          GPIOC
#define ADC1_GPIO_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

#define ADC2_CHANNEL       ADC_CHANNEL_11
#define ADC2_Pin           GPIO_PIN_1
#define ADC2_Port          GPIOC
#define ADC2_GPIO_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

#define ADC3_CHANNEL       ADC_CHANNEL_0
#define ADC3_Pin           GPIO_PIN_0
#define ADC3_Port          GPIOA
#define ADC3_GPIO_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

/* DRV Gate drive */
#define ENABLE_PIN 		GPIOA, GPIO_PIN_11  // Enable gate drive pin. (FIX, cannot be same as CAN!)
#define DRV_SPI			hspi1				// DRV SPI handle
#define DRV_CS			GPIOA, GPIO_PIN_4	// DRV CS pin

/* SPI encoder */
#define ENC_SPI			hspi3				// Encoder SPI handle
#define ENC_CS			GPIOA, GPIO_PIN_15	// Encoder SPI CS pin
#define ENC_CPR			65536				// Encoder counts per revolution
#define INV_CPR			1.0f/ENC_CPR
#define ENC_READ_WORD	0x00				// Encoder read command

/* Misc. GPIO */
#define LED         	GPIOC, GPIO_PIN_5	// LED Pin

/* CAN */
#define CAN_H			hcan1				// CAN handle
#define CAN_TX_Pin GPIO_PIN_1
#define CAN_RX_Pin GPIO_PIN_0
#define CAN_GPIO_Port GPIOD
#define CAN_CLK_ENABLE() __HAL_RCC_CAN1_CLK_ENABLE()
#define CAN_GPIO_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE();

/* SPI */
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_Port GPIOA

#define SPI3_SCK_Pin GPIO_PIN_10
#define SPI3_MISO_Pin GPIO_PIN_11
#define SPI3_MOSI_Pin GPIO_PIN_1
#define SPI3_Port GPIOC

/* Other hardware-related constants */
#define I_SCALE 			0.0201416f  // Amps per A/D Count at 40X amplifier gain
#define V_SCALE 			0.0128906f    // Bus volts per A/D Count
#define DTC_MAX 			0.94f          	// Max duty cycle
#define DTC_MIN 			0.0f          	// Min duty cycle
#define DTC_COMP 			0.000f          // deadtime compensation (100 ns / 25 us)
#define DT					.000025f		// Loop period
#define EN_ENC_LINEARIZATION 1				// Enable/disable encoder linearization


/* Current controller */
#define L_D .000003f				// D axis inductance
#define L_Q .000003f				// Q axis inductance
#define K_D .05f                    // Loop gain,  Volts/Amp
#define K_Q .05f                    // Loop gain,  Volts/Amp
#define K_SCALE 0.0001f             // K_loop/Loop BW (Hz) 0.0042
#define KI_D 0.045f                // PI zero, in radians per sample
#define KI_Q 0.045f                // PI zero, in radians per sample
#define OVERMODULATION 1.15f        // 1.0 = no overmodulation
#define CURRENT_FILT_ALPHA	.1f	// 1st order d/q current filter (not used in control)
#define VBUS_FILT_ALPHA		.1f		// 1st order bus voltage filter

#define D_INT_LIM V_BUS/(K_D*KI_D)  // Amps*samples
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  // Amps*samples

#define USARTx  USART3
#define USART_TX_Pin GPIO_PIN_8
#define USART_RX_Pin GPIO_PIN_9
#define USART_GPIO_Port GPIOD
#define USARTx_CLK_ENABLE()  __HAL_RCC_USART3_CLK_ENABLE()
#define USARTx_CLK_DISABLE()  __HAL_RCC_USART3_CLK_DISABLE()
#define USARTx_GPIO_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#define USARTx_IRQ USART3_IRQn
#define USARTx_AF GPIO_AF7_USART3

#endif
