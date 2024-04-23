// Copyright (c) 2021-2023 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

import { DisplayComponent, FSComponent, NodeReference, VNode, Subscribable } from '@microsoft/msfs-sdk';
import { ArincEventBus } from '@flybywiresim/fbw-sdk';

import { DmcLogicEvents } from '../MsfsAvionicsCommon/providers/DmcPublisher';
import { Arinc429Values } from './shared/ArincValueProvider';

interface HorizontalTapeProps {
    displayRange: number;
    valueSpacing: number;
    distanceSpacing: number;
    type: 'horizon' | 'headingTape'
    bus: ArincEventBus;
    yOffset?: Subscribable<number>;
}
export class HorizontalTape extends DisplayComponent<HorizontalTapeProps> {
    private refElement = FSComponent.createRef<SVGGElement>();

    private tapeOffset=0;

    private tickNumberRefs: NodeReference<SVGTextElement>[] = [];

    private currentDrawnHeading = 0;

    private yOffset = 0;

    private buildHorizonTicks():{ticks: SVGPathElement[], labels?: SVGTextElement[]} {
        const result = { ticks: [] as SVGPathElement[], labels: [] as SVGTextElement[] };

        result.ticks.push(<path transform="translate(0 0)" class="NormalStroke White" d="m68.906 80.823v1.8" />);

        for (let i = 0; i < 6; i++) {
            const headingOffset = (1 + i) * this.props.valueSpacing;
            const dX = this.props.distanceSpacing / this.props.valueSpacing * headingOffset;

            if (headingOffset % 10 === 0) {
                result.ticks.push(<path class="NormalStroke White" d="m68.906 80.823v1.8" transform={`translate(${dX} 0)`} />);
                result.ticks.unshift(<path class="NormalStroke White" d="m68.906 80.823v1.8" transform={`translate(${-dX} 0)`} />);
            }
        }

        return result;
    }

    private buildHeadingTicks(): { ticks: SVGLineElement[], labels: SVGTextElement[] } {
        const result = {
            ticks: [] as SVGLineElement[],
            labels: [] as SVGTextElement[],
        };

        const tickLength = 7;
        let textRef = FSComponent.createRef<SVGTextElement>();

        result.ticks.push(<path class="NormalStroke Green" d={`m640 512v${tickLength}`} transform="translate(0 0)" />);

        result.labels.push(
            <text
                id="HeadingLabel"
                class="Green MiddleAlign FontSmallest"
                ref={textRef}
                x="640"
                y="540"
                transform={`translate(${0} 0)`}
            >
                360

            </text>,

        );
        this.tickNumberRefs.push(textRef);

        for (let i = 0; i < 6; i++) {
            const headingOffset = (1 + i) * this.props.valueSpacing;
            const dX = this.props.distanceSpacing / this.props.valueSpacing * headingOffset;

            // if (headingOffset % 10 === 0) {
            result.ticks.push(<path class="NormalStroke Green" d={`m640 512v${tickLength}`} style={`transform: translate3d(${dX}px, 0px, 0px)`} />);
            result.ticks.unshift(<path class="NormalStroke Green" d={`m640 512v${tickLength}`} style={`transform: translate3d(${-dX}px, 0px, 0px)`} />);
            // } else {
            //     result.ticks.push(<path class="NormalStroke Green" d={`m512 384v${tickLength * 0.42}`} style={`transform: translate3d(${dX}px, 0px, 0px)`} />);
            //     result.ticks.unshift(<path class="NormalStroke Green" d={`m512 384v${tickLength * 0.42}`} style={`transform: translate3d(${-dX}px, 0px, 0px)`} />);
            // }

            if (headingOffset % 10 === 0) {
                textRef = FSComponent.createRef<SVGTextElement>();

                result.labels.unshift(
                    <text
                        id="HeadingLabel"
                        class="Green MiddleAlign FontSmallest"
                        ref={textRef}
                        x="640"
                        y="540"
                        style={`transform: translate3d(${-dX}px, 0px, 0px)`}
                    >
                        {headingOffset}

                    </text>,
                );
                this.tickNumberRefs.unshift(textRef);
                textRef = FSComponent.createRef<SVGTextElement>();
                result.labels.push(
                    <text
                        id="HeadingLabel"
                        class="Green MiddleAlign FontSmallest"
                        ref={textRef}
                        x="640"
                        y="540"
                        style={`transform: translate3d(${dX}px, 0px, 0px)`}
                    >
                        {(360 - headingOffset)}

                    </text>,
                );
                this.tickNumberRefs.push(textRef);
            }
        }

        return result;
    }

    onAfterRender(node: VNode): void {
        super.onAfterRender(node);

        const pf = this.props.bus.getArincSubscriber<Arinc429Values & DmcLogicEvents>();

        this.props.yOffset?.sub((yOffset) => {
            this.yOffset = yOffset;
            this.refElement.instance.style.transform = `translate3d(${this.tapeOffset}px, ${yOffset}px, 0px)`;
        });

        pf.on('heading').withArinc429Precision(2).handle((newVal) => {
            const tapeOffset = -newVal.value % 10 * this.props.distanceSpacing / this.props.valueSpacing;

            if (newVal.value / 10 >= this.currentDrawnHeading + 1 || newVal.value / 10 <= this.currentDrawnHeading) {
                this.currentDrawnHeading = Math.floor(newVal.value / 10);

                const start = 330 + (this.currentDrawnHeading) * 10;

                this.tickNumberRefs.forEach((t, index) => {
                    const scrollerValue = t.instance;
                    if (scrollerValue !== null) {
                        const hdg = (start + index * 10) % 360;
                        if (hdg % 10 === 0) {
                            const content = hdg !== 0 ? (hdg / 10).toFixed(0) : '0';
                            if (scrollerValue.textContent !== content) {
                                scrollerValue.textContent = content;
                            }
                        } else {
                            scrollerValue.textContent = '';
                        }
                        // if (hdg % 30 === 0) {
                        //     scrollerValue.classList.remove('FontSmallest');
                        //     scrollerValue.classList.add('FontMedium');
                        // } else {
                        //     scrollerValue.classList.add('FontSmallest');
                        //     scrollerValue.classList.remove('FontMedium');
                        // }
                    }
                });
            }
            this.tapeOffset = tapeOffset;

            this.refElement.instance.style.transform = `translate3d(${tapeOffset}px, ${this.yOffset}px, 0px)`;
        });
    }

    render(): VNode {
        const tapeContent = this.props.type === 'horizon' ? this.buildHorizonTicks() : this.buildHeadingTicks();

        return (

            <g id="HeadingTick" ref={this.refElement}>

                {tapeContent.ticks}
                {this.props.type === 'headingTape' && tapeContent.labels}

            </g>

        );
    }
}